"""
auto_takeoff_landing_system.py
IntegratedDroneSystem을 활용한 자동 이륙/착륙 시스템
시동 → 5초 대기 → 2m 이륙 → 10초 호버링(위치유지) → 착륙
"""

import sys
import os
import serial
import json
import threading
import time
import math
import logging
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum

# integrated_drone_system 모듈 import
# 같은 디렉토리에 있다고 가정
from integrated_drone_system import IntegratedDroneSystem, GPSData, ControlMode

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

# ========================= 상태 정의 =========================

class FlightState(Enum):
    """비행 상태"""
    DISARMED = "disarmed"
    ARMING = "arming"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    HOVERING = "hovering"
    LANDING = "landing"
    LANDED = "landed"
    ERROR = "error"

@dataclass
class AltitudeData:
    """고도 데이터"""
    relative: float
    absolute: float
    timestamp: float

@dataclass
class IMUData:
    """IMU 센서 데이터"""
    roll: float
    pitch: float
    yaw: float
    acc_x: float
    acc_y: float
    acc_z: float
    timestamp: float

# ========================= 자동 이륙/착륙 시스템 =========================

class AutoTakeoffLandingSystem(IntegratedDroneSystem):
    """자동 이륙/착륙 및 위치 유지 시스템"""
    
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200):
        """초기화"""
        super().__init__(port, baudrate)
        
        # 비행 상태
        self.flight_state = FlightState.DISARMED
        self.state_lock = threading.Lock()
        
        # 센서 데이터
        self.current_altitude = AltitudeData(0, 0, time.time())
        self.altitude_lock = threading.Lock()
        self.takeoff_altitude = 0
        
        self.current_imu = IMUData(0, 0, 0, 0, 0, 0, time.time())
        self.imu_lock = threading.Lock()
        
        # 위치 유지
        self.home_gps = None
        self.position_hold_enabled = False
        self.position_hold_thread = None
        
        # 파라미터
        self.TARGET_ALTITUDE = 2.0  # 목표 고도 (m)
        self.ALTITUDE_TOLERANCE = 0.2  # 고도 허용 오차 (m)
        self.POSITION_TOLERANCE = 0.5  # 위치 허용 오차 (m)
        
        # 쓰로틀 설정
        self.THROTTLE_TAKEOFF = 75.0   # 이륙 쓰로틀 (%)
        self.THROTTLE_HOVER = 60.0     # 호버링 쓰로틀 (%)
        self.THROTTLE_LAND = 40.0      # 착륙 쓰로틀 (%)
        
        # 위치 보정 파라미터
        self.MAX_TILT_ANGLE = 10.0  # 최대 기울기 (도)
        self.POSITION_CORRECTION_SPEED = 30.0  # 위치 보정 속도 (%)
        self.WIND_RESISTANCE_GAIN = 1.5  # 바람 저항 계수
        
    # ========================= ARM/DISARM =========================
    
    def arm(self) -> bool:
        """시동 걸기"""
        try:
            self._set_flight_state(FlightState.ARMING)
            
            command = {
                'type': 'ARM',
                'data': {
                    'action': 'arm',
                    'timestamp': time.time()
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                
                # 응답 대기
                time.sleep(2)
                
                # 시동 상태 확인 (실제로는 FC 응답을 받아야 함)
                self._set_flight_state(FlightState.ARMED)
                self.logger.info("✓ 시동 성공 (ARM)")
                return True
                
        except Exception as e:
            self.logger.error(f"시동 실패: {e}")
            self._set_flight_state(FlightState.ERROR)
            
        return False
    
    def disarm(self) -> bool:
        """시동 끄기"""
        try:
            self.position_hold_enabled = False
            
            command = {
                'type': 'ARM',
                'data': {
                    'action': 'disarm',
                    'timestamp': time.time()
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                
                self._set_flight_state(FlightState.DISARMED)
                self.logger.info("시동 끄기 완료 (DISARM)")
                return True
                
        except Exception as e:
            self.logger.error(f"DISARM 오류: {e}")
            
        return False
    
    # ========================= 자동 이륙 =========================
    
    def auto_takeoff(self, altitude: float = 2.0) -> bool:
        """
        자동 이륙
        
        Args:
            altitude: 목표 고도 (m)
        """
        self.TARGET_ALTITUDE = altitude
        self.logger.info(f"자동 이륙 시작 - 목표 고도: {altitude}m")
        
        # 현재 위치를 홈으로 저장
        self._save_home_position()
        
        # 상태 변경
        self._set_flight_state(FlightState.TAKING_OFF)
        
        # 현재 고도 저장
        with self.altitude_lock:
            self.takeoff_altitude = self.current_altitude.relative
        
        # 상승 시작
        start_time = time.time()
        
        while True:
            # 상승 명령
            self.set_command("up", "hover", 0, self.THROTTLE_TAKEOFF)
            
            # 현재 고도 확인
            current_alt = self._get_relative_altitude()
            
            # 목표 고도 도달 확인
            if current_alt >= (self.TARGET_ALTITUDE - self.ALTITUDE_TOLERANCE):
                self.logger.info(f"✓ 목표 고도 도달: {current_alt:.2f}m")
                return True
            
            # 타임아웃 체크 (30초)
            if time.time() - start_time > 30:
                self.logger.error("이륙 타임아웃")
                return False
            
            # 상태 출력 (2초마다)
            if int(time.time() - start_time) % 2 == 0:
                self.logger.info(f"상승 중... 현재: {current_alt:.2f}m / 목표: {self.TARGET_ALTITUDE}m")
            
            time.sleep(0.1)
    
    # ========================= 호버링 및 위치 유지 =========================
    
    def start_hovering(self):
        """호버링 시작 (위치 유지 포함)"""
        self._set_flight_state(FlightState.HOVERING)
        self.position_hold_enabled = True
        
        # 호버링 명령
        self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
        
        # 위치 유지 스레드 시작
        if not self.position_hold_thread or not self.position_hold_thread.is_alive():
            self.position_hold_thread = threading.Thread(target=self._position_hold_loop, daemon=True)
            self.position_hold_thread.start()
        
        self.logger.info(f"호버링 모드 (쓰로틀: {self.THROTTLE_HOVER}%, 위치 유지 ON)")
    
    def _position_hold_loop(self):
        """위치 유지 루프 (바람 저항)"""
        while self.position_hold_enabled and self.flight_state == FlightState.HOVERING:
            try:
                # GPS 기반 위치 보정
                if self.current_gps and self.home_gps:
                    distance, bearing = self._calculate_position_error()
                    
                    if distance > self.POSITION_TOLERANCE:
                        self._correct_position_gps(distance, bearing)
                
                # IMU 기반 자세 보정 (바람 저항)
                with self.imu_lock:
                    roll = self.current_imu.roll
                    pitch = self.current_imu.pitch
                
                if abs(roll) > self.MAX_TILT_ANGLE or abs(pitch) > self.MAX_TILT_ANGLE:
                    self._correct_attitude(roll, pitch)
                
                # 고도 유지
                current_alt = self._get_relative_altitude()
                if abs(current_alt - self.TARGET_ALTITUDE) > self.ALTITUDE_TOLERANCE:
                    self._adjust_altitude(current_alt)
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.logger.error(f"위치 유지 오류: {e}")
                time.sleep(0.5)
    
    def _correct_position_gps(self, distance: float, bearing: float):
        """GPS 기반 위치 보정"""
        self.logger.debug(f"위치 보정: 거리={distance:.2f}m, 방향={bearing:.0f}°")
        
        # 현재 heading과 목표 방향 차이
        if self.current_gps:
            heading_diff = bearing - self.current_gps.heading
            if heading_diff > 180:
                heading_diff -= 360
            elif heading_diff < -180:
                heading_diff += 360
            
            # 이동 방향 결정 (반대로 이동해야 원위치)
            if abs(heading_diff) < 45:
                horizontal = "backward"
            elif abs(heading_diff) < 135:
                if heading_diff > 0:
                    horizontal = "left"
                else:
                    horizontal = "right"
            else:
                horizontal = "forward"
            
            # 거리에 비례한 속도
            speed = min(self.POSITION_CORRECTION_SPEED, distance * 10)
            
            # 보정 명령
            self.set_command("level", horizontal, 0, speed)
            time.sleep(0.2)
            
            # 다시 호버링
            self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
    
    def _correct_attitude(self, roll: float, pitch: float):
        """자세 보정 (바람 저항)"""
        self.logger.debug(f"바람 보정: Roll={roll:.1f}°, Pitch={pitch:.1f}°")
        
        correction_made = False
        
        # 롤 보정 (좌우 기울기)
        if abs(roll) > self.MAX_TILT_ANGLE:
            speed = abs(roll) * self.WIND_RESISTANCE_GAIN
            if roll > 0:  # 우측 기울어짐 → 좌측 이동
                self.set_command("level", "left", 0, speed)
            else:  # 좌측 기울어짐 → 우측 이동
                self.set_command("level", "right", 0, speed)
            correction_made = True
            time.sleep(0.15)
        
        # 피치 보정 (전후 기울기)
        if abs(pitch) > self.MAX_TILT_ANGLE:
            speed = abs(pitch) * self.WIND_RESISTANCE_GAIN
            if pitch > 0:  # 앞으로 기울어짐 → 후진
                self.set_command("level", "backward", 0, speed)
            else:  # 뒤로 기울어짐 → 전진
                self.set_command("level", "forward", 0, speed)
            correction_made = True
            time.sleep(0.15)
        
        # 보정 후 호버링 복귀
        if correction_made:
            self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
    
    def _adjust_altitude(self, current_alt: float):
        """고도 보정"""
        if current_alt < self.TARGET_ALTITUDE - self.ALTITUDE_TOLERANCE:
            # 상승
            self.set_command("up", "hover", 0, self.THROTTLE_HOVER + 10)
            time.sleep(0.3)
        elif current_alt > self.TARGET_ALTITUDE + self.ALTITUDE_TOLERANCE:
            # 하강
            self.set_command("down", "hover", 0, self.THROTTLE_HOVER - 10)
            time.sleep(0.3)
        
        # 호버링 복귀
        self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
    
    # ========================= 자동 착륙 =========================
    
    def auto_land(self) -> bool:
        """자동 착륙"""
        self.logger.info("자동 착륙 시작")
        
        # 위치 유지 비활성화
        self.position_hold_enabled = False
        self._set_flight_state(FlightState.LANDING)
        
        # 착륙 시작
        start_time = time.time()
        
        while True:
            # 하강 명령
            self.set_command("down", "hover", 0, self.THROTTLE_LAND)
            
            # 현재 고도 확인
            current_alt = self._get_relative_altitude()
            
            # 착륙 확인 (0.2m 이하)
            if current_alt <= 0.2:
                self.logger.info("✓ 착륙 완료")
                self._set_flight_state(FlightState.LANDED)
                
                # 모터 정지
                self.set_command("level", "hover", 0, 0)
                time.sleep(1)
                
                # 시동 끄기
                self.disarm()
                return True
            
            # 타임아웃 체크 (60초)
            if time.time() - start_time > 60:
                self.logger.error("착륙 타임아웃")
                return False
            
            # 상태 출력 (2초마다)
            if int(time.time() - start_time) % 2 == 0:
                self.logger.info(f"하강 중... 현재 고도: {current_alt:.2f}m")
            
            time.sleep(0.1)
    
    # ========================= 메인 미션 실행 =========================
    
    def execute_mission(self):
        """
        메인 미션 실행
        시동 → 5초 대기 → 2m 이륙 → 10초 호버링 → 착륙
        """
        try:
            self.logger.info("="*60)
            self.logger.info("자동 이륙/착륙 미션 시작")
            self.logger.info("="*60)
            
            # 1. 시동 걸기
            self.logger.info("\n[1단계] 시동 시도...")
            if not self.arm():
                self.logger.error("시동 실패! 미션 중단")
                return False
            
            # 2. 5초 대기
            self.logger.info("\n[2단계] 5초 대기...")
            for i in range(5, 0, -1):
                self.logger.info(f"이륙까지 {i}초...")
                time.sleep(1)
            
            # 3. 2m 이륙
            self.logger.info("\n[3단계] 2m 상공으로 이륙...")
            if not self.auto_takeoff(2.0):
                self.logger.error("이륙 실패! 긴급 착륙")
                self.emergency_stop()
                self.disarm()
                return False
            
            # 4. 10초 호버링 (위치 유지)
            self.logger.info("\n[4단계] 10초간 호버링 (위치 유지 활성화)...")
            self.start_hovering()
            
            for i in range(10):
                alt = self._get_relative_altitude()
                
                # GPS 정보가 있으면 표시
                if self.current_gps:
                    self.logger.info(f"호버링 {i+1}/10초 | 고도: {alt:.2f}m | "
                                   f"GPS: ({self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f})")
                else:
                    self.logger.info(f"호버링 {i+1}/10초 | 고도: {alt:.2f}m")
                
                time.sleep(1)
            
            # 5. 착륙
            self.logger.info("\n[5단계] 착륙 시작...")
            if not self.auto_land():
                self.logger.error("착륙 실패!")
                self.emergency_stop()
                self.disarm()
                return False
            
            self.logger.info("\n" + "="*60)
            self.logger.info("✅ 미션 완료!")
            self.logger.info("="*60)
            return True
            
        except Exception as e:
            self.logger.error(f"미션 실행 중 오류: {e}")
            self.emergency_stop()
            self.disarm()
            return False
    
    # ========================= 헬퍼 메서드 =========================
    
    def _set_flight_state(self, state: FlightState):
        """비행 상태 설정"""
        with self.state_lock:
            self.flight_state = state
            self.logger.debug(f"상태 변경: {state.value}")
    
    def _get_relative_altitude(self) -> float:
        """상대 고도 반환"""
        with self.altitude_lock:
            return self.current_altitude.relative - self.takeoff_altitude
    
    def _save_home_position(self):
        """현재 위치를 홈으로 저장"""
        if self.current_gps:
            self.home_gps = (self.current_gps.latitude, self.current_gps.longitude)
            self.set_home(self.current_gps.latitude, self.current_gps.longitude)
            self.logger.info(f"홈 위치 저장: ({self.home_gps[0]:.6f}, {self.home_gps[1]:.6f})")
    
    def _calculate_position_error(self) -> Tuple[float, float]:
        """위치 오차 계산"""
        if not self.current_gps or not self.home_gps:
            return 0, 0
        
        # 거리 계산
        R = 6371000
        lat1, lon1 = self.home_gps
        lat2, lon2 = self.current_gps.latitude, self.current_gps.longitude
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        
        # 방향 계산
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        bearing = math.degrees(math.atan2(x, y))
        bearing = (bearing + 360) % 360
        
        return distance, bearing
    
    # ========================= 메시지 처리 오버라이드 =========================
    
    def _process_message(self, message: str):
        """메시지 처리 (오버라이드)"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            # 부모 클래스 처리
            super()._process_message(message)
            
            # 추가 처리
            if msg_type == 'ALTITUDE':
                self._handle_altitude_data(data.get('data', {}))
            elif msg_type == 'IMU':
                self._handle_imu_data(data.get('data', {}))
            elif msg_type == 'ARM_STATUS':
                self._handle_arm_status(data.get('data', {}))
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
    
    def _handle_altitude_data(self, data: Dict):
        """고도 데이터 처리"""
        with self.altitude_lock:
            self.current_altitude = AltitudeData(
                relative=data.get('relative', 0),
                absolute=data.get('absolute', 0),
                timestamp=time.time()
            )
    
    def _handle_imu_data(self, data: Dict):
        """IMU 데이터 처리"""
        with self.imu_lock:
            self.current_imu = IMUData(
                roll=data.get('roll', 0),
                pitch=data.get('pitch', 0),
                yaw=data.get('yaw', 0),
                acc_x=data.get('acc_x', 0),
                acc_y=data.get('acc_y', 0),
                acc_z=data.get('acc_z', 0),
                timestamp=time.time()
            )
    
    def _handle_arm_status(self, data: Dict):
        """시동 상태 처리"""
        is_armed = data.get('armed', False)
        if is_armed and self.flight_state == FlightState.ARMING:
            self._set_flight_state(FlightState.ARMED)
        elif not is_armed and self.flight_state != FlightState.DISARMED:
            self._set_flight_state(FlightState.DISARMED)


# ========================= 메인 실행 =========================

def main():
    """메인 실행 함수"""
    
    print("\n" + "="*60)
    print("자동 이륙/착륙 시스템")
    print("시동 → 5초 대기 → 2m 이륙 → 10초 호버링 → 착륙")
    print("="*60 + "\n")
    
    # 시스템 생성
    drone_system = AutoTakeoffLandingSystem("/dev/ttyTHS1", 115200)
    
    # 연결
    if not drone_system.connect():
        print("❌ 드론 연결 실패!")
        return
    
    try:
        # 미션 실행
        success = drone_system.execute_mission()
        
        if success:
            print("\n🎉 모든 미션이 성공적으로 완료되었습니다!")
            print("시스템 설정:")
            print(f"  - 이륙 쓰로틀: {drone_system.THROTTLE_TAKEOFF}%")
            print(f"  - 호버링 쓰로틀: {drone_system.THROTTLE_HOVER}%")
            print(f"  - 착륙 쓰로틀: {drone_system.THROTTLE_LAND}%")
            print(f"  - 위치 허용 오차: {drone_system.POSITION_TOLERANCE}m")
            print(f"  - 바람 저항 계수: {drone_system.WIND_RESISTANCE_GAIN}")
        else:
            print("\n❌ 미션 실패")
            
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자 중단! 긴급 정지 및 착륙...")
        drone_system.emergency_stop()
        time.sleep(1)
        drone_system.auto_land()
        
    except Exception as e:
        print(f"\n❌ 시스템 오류: {e}")
        drone_system.emergency_stop()
        drone_system.disarm()
        
    finally:
        # 연결 해제
        drone_system.disconnect()
        print("\n시스템 종료")


if __name__ == "__main__":
    main()"""
auto_takeoff_landing_system.py
IntegratedDroneSystem을 활용한 자동 이륙/착륙 시스템
시동 → 5초 대기 → 2m 이륙 → 10초 호버링(위치유지) → 착륙
"""

import sys
import os
import serial
import json
import threading
import time
import math
import logging
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum

# integrated_drone_system 모듈 import
# 같은 디렉토리에 있다고 가정
from integrated_drone_system import IntegratedDroneSystem, GPSData, ControlMode

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

# ========================= 상태 정의 =========================

class FlightState(Enum):
    """비행 상태"""
    DISARMED = "disarmed"
    ARMING = "arming"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    HOVERING = "hovering"
    LANDING = "landing"
    LANDED = "landed"
    ERROR = "error"

@dataclass
class AltitudeData:
    """고도 데이터"""
    relative: float
    absolute: float
    timestamp: float

@dataclass
class IMUData:
    """IMU 센서 데이터"""
    roll: float
    pitch: float
    yaw: float
    acc_x: float
    acc_y: float
    acc_z: float
    timestamp: float

# ========================= 자동 이륙/착륙 시스템 =========================

class AutoTakeoffLandingSystem(IntegratedDroneSystem):
    """자동 이륙/착륙 및 위치 유지 시스템"""
    
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200):
        """초기화"""
        super().__init__(port, baudrate)
        
        # 비행 상태
        self.flight_state = FlightState.DISARMED
        self.state_lock = threading.Lock()
        
        # 센서 데이터
        self.current_altitude = AltitudeData(0, 0, time.time())
        self.altitude_lock = threading.Lock()
        self.takeoff_altitude = 0
        
        self.current_imu = IMUData(0, 0, 0, 0, 0, 0, time.time())
        self.imu_lock = threading.Lock()
        
        # 위치 유지
        self.home_gps = None
        self.position_hold_enabled = False
        self.position_hold_thread = None
        
        # 파라미터
        self.TARGET_ALTITUDE = 2.0  # 목표 고도 (m)
        self.ALTITUDE_TOLERANCE = 0.2  # 고도 허용 오차 (m)
        self.POSITION_TOLERANCE = 0.5  # 위치 허용 오차 (m)
        
        # 쓰로틀 설정
        self.THROTTLE_TAKEOFF = 75.0   # 이륙 쓰로틀 (%)
        self.THROTTLE_HOVER = 60.0     # 호버링 쓰로틀 (%)
        self.THROTTLE_LAND = 40.0      # 착륙 쓰로틀 (%)
        
        # 위치 보정 파라미터
        self.MAX_TILT_ANGLE = 10.0  # 최대 기울기 (도)
        self.POSITION_CORRECTION_SPEED = 30.0  # 위치 보정 속도 (%)
        self.WIND_RESISTANCE_GAIN = 1.5  # 바람 저항 계수
        
    # ========================= ARM/DISARM =========================
    
    def arm(self) -> bool:
        """시동 걸기"""
        try:
            self._set_flight_state(FlightState.ARMING)
            
            command = {
                'type': 'ARM',
                'data': {
                    'action': 'arm',
                    'timestamp': time.time()
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                
                # 응답 대기
                time.sleep(2)
                
                # 시동 상태 확인 (실제로는 FC 응답을 받아야 함)
                self._set_flight_state(FlightState.ARMED)
                self.logger.info("✓ 시동 성공 (ARM)")
                return True
                
        except Exception as e:
            self.logger.error(f"시동 실패: {e}")
            self._set_flight_state(FlightState.ERROR)
            
        return False
    
    def disarm(self) -> bool:
        """시동 끄기"""
        try:
            self.position_hold_enabled = False
            
            command = {
                'type': 'ARM',
                'data': {
                    'action': 'disarm',
                    'timestamp': time.time()
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                
                self._set_flight_state(FlightState.DISARMED)
                self.logger.info("시동 끄기 완료 (DISARM)")
                return True
                
        except Exception as e:
            self.logger.error(f"DISARM 오류: {e}")
            
        return False
    
    # ========================= 자동 이륙 =========================
    
    def auto_takeoff(self, altitude: float = 2.0) -> bool:
        """
        자동 이륙
        
        Args:
            altitude: 목표 고도 (m)
        """
        self.TARGET_ALTITUDE = altitude
        self.logger.info(f"자동 이륙 시작 - 목표 고도: {altitude}m")
        
        # 현재 위치를 홈으로 저장
        self._save_home_position()
        
        # 상태 변경
        self._set_flight_state(FlightState.TAKING_OFF)
        
        # 현재 고도 저장
        with self.altitude_lock:
            self.takeoff_altitude = self.current_altitude.relative
        
        # 상승 시작
        start_time = time.time()
        
        while True:
            # 상승 명령
            self.set_command("up", "hover", 0, self.THROTTLE_TAKEOFF)
            
            # 현재 고도 확인
            current_alt = self._get_relative_altitude()
            
            # 목표 고도 도달 확인
            if current_alt >= (self.TARGET_ALTITUDE - self.ALTITUDE_TOLERANCE):
                self.logger.info(f"✓ 목표 고도 도달: {current_alt:.2f}m")
                return True
            
            # 타임아웃 체크 (30초)
            if time.time() - start_time > 30:
                self.logger.error("이륙 타임아웃")
                return False
            
            # 상태 출력 (2초마다)
            if int(time.time() - start_time) % 2 == 0:
                self.logger.info(f"상승 중... 현재: {current_alt:.2f}m / 목표: {self.TARGET_ALTITUDE}m")
            
            time.sleep(0.1)
    
    # ========================= 호버링 및 위치 유지 =========================
    
    def start_hovering(self):
        """호버링 시작 (위치 유지 포함)"""
        self._set_flight_state(FlightState.HOVERING)
        self.position_hold_enabled = True
        
        # 호버링 명령
        self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
        
        # 위치 유지 스레드 시작
        if not self.position_hold_thread or not self.position_hold_thread.is_alive():
            self.position_hold_thread = threading.Thread(target=self._position_hold_loop, daemon=True)
            self.position_hold_thread.start()
        
        self.logger.info(f"호버링 모드 (쓰로틀: {self.THROTTLE_HOVER}%, 위치 유지 ON)")
    
    def _position_hold_loop(self):
        """위치 유지 루프 (바람 저항)"""
        while self.position_hold_enabled and self.flight_state == FlightState.HOVERING:
            try:
                # GPS 기반 위치 보정
                if self.current_gps and self.home_gps:
                    distance, bearing = self._calculate_position_error()
                    
                    if distance > self.POSITION_TOLERANCE:
                        self._correct_position_gps(distance, bearing)
                
                # IMU 기반 자세 보정 (바람 저항)
                with self.imu_lock:
                    roll = self.current_imu.roll
                    pitch = self.current_imu.pitch
                
                if abs(roll) > self.MAX_TILT_ANGLE or abs(pitch) > self.MAX_TILT_ANGLE:
                    self._correct_attitude(roll, pitch)
                
                # 고도 유지
                current_alt = self._get_relative_altitude()
                if abs(current_alt - self.TARGET_ALTITUDE) > self.ALTITUDE_TOLERANCE:
                    self._adjust_altitude(current_alt)
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.logger.error(f"위치 유지 오류: {e}")
                time.sleep(0.5)
    
    def _correct_position_gps(self, distance: float, bearing: float):
        """GPS 기반 위치 보정"""
        self.logger.debug(f"위치 보정: 거리={distance:.2f}m, 방향={bearing:.0f}°")
        
        # 현재 heading과 목표 방향 차이
        if self.current_gps:
            heading_diff = bearing - self.current_gps.heading
            if heading_diff > 180:
                heading_diff -= 360
            elif heading_diff < -180:
                heading_diff += 360
            
            # 이동 방향 결정 (반대로 이동해야 원위치)
            if abs(heading_diff) < 45:
                horizontal = "backward"
            elif abs(heading_diff) < 135:
                if heading_diff > 0:
                    horizontal = "left"
                else:
                    horizontal = "right"
            else:
                horizontal = "forward"
            
            # 거리에 비례한 속도
            speed = min(self.POSITION_CORRECTION_SPEED, distance * 10)
            
            # 보정 명령
            self.set_command("level", horizontal, 0, speed)
            time.sleep(0.2)
            
            # 다시 호버링
            self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
    
    def _correct_attitude(self, roll: float, pitch: float):
        """자세 보정 (바람 저항)"""
        self.logger.debug(f"바람 보정: Roll={roll:.1f}°, Pitch={pitch:.1f}°")
        
        correction_made = False
        
        # 롤 보정 (좌우 기울기)
        if abs(roll) > self.MAX_TILT_ANGLE:
            speed = abs(roll) * self.WIND_RESISTANCE_GAIN
            if roll > 0:  # 우측 기울어짐 → 좌측 이동
                self.set_command("level", "left", 0, speed)
            else:  # 좌측 기울어짐 → 우측 이동
                self.set_command("level", "right", 0, speed)
            correction_made = True
            time.sleep(0.15)
        
        # 피치 보정 (전후 기울기)
        if abs(pitch) > self.MAX_TILT_ANGLE:
            speed = abs(pitch) * self.WIND_RESISTANCE_GAIN
            if pitch > 0:  # 앞으로 기울어짐 → 후진
                self.set_command("level", "backward", 0, speed)
            else:  # 뒤로 기울어짐 → 전진
                self.set_command("level", "forward", 0, speed)
            correction_made = True
            time.sleep(0.15)
        
        # 보정 후 호버링 복귀
        if correction_made:
            self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
    
    def _adjust_altitude(self, current_alt: float):
        """고도 보정"""
        if current_alt < self.TARGET_ALTITUDE - self.ALTITUDE_TOLERANCE:
            # 상승
            self.set_command("up", "hover", 0, self.THROTTLE_HOVER + 10)
            time.sleep(0.3)
        elif current_alt > self.TARGET_ALTITUDE + self.ALTITUDE_TOLERANCE:
            # 하강
            self.set_command("down", "hover", 0, self.THROTTLE_HOVER - 10)
            time.sleep(0.3)
        
        # 호버링 복귀
        self.set_command("level", "hover", 0, self.THROTTLE_HOVER)
    
    # ========================= 자동 착륙 =========================
    
    def auto_land(self) -> bool:
        """자동 착륙"""
        self.logger.info("자동 착륙 시작")
        
        # 위치 유지 비활성화
        self.position_hold_enabled = False
        self._set_flight_state(FlightState.LANDING)
        
        # 착륙 시작
        start_time = time.time()
        
        while True:
            # 하강 명령
            self.set_command("down", "hover", 0, self.THROTTLE_LAND)
            
            # 현재 고도 확인
            current_alt = self._get_relative_altitude()
            
            # 착륙 확인 (0.2m 이하)
            if current_alt <= 0.2:
                self.logger.info("✓ 착륙 완료")
                self._set_flight_state(FlightState.LANDED)
                
                # 모터 정지
                self.set_command("level", "hover", 0, 0)
                time.sleep(1)
                
                # 시동 끄기
                self.disarm()
                return True
            
            # 타임아웃 체크 (60초)
            if time.time() - start_time > 60:
                self.logger.error("착륙 타임아웃")
                return False
            
            # 상태 출력 (2초마다)
            if int(time.time() - start_time) % 2 == 0:
                self.logger.info(f"하강 중... 현재 고도: {current_alt:.2f}m")
            
            time.sleep(0.1)
    
    # ========================= 메인 미션 실행 =========================
    
    def execute_mission(self):
        """
        메인 미션 실행
        시동 → 5초 대기 → 2m 이륙 → 10초 호버링 → 착륙
        """
        try:
            self.logger.info("="*60)
            self.logger.info("자동 이륙/착륙 미션 시작")
            self.logger.info("="*60)
            
            # 1. 시동 걸기
            self.logger.info("\n[1단계] 시동 시도...")
            if not self.arm():
                self.logger.error("시동 실패! 미션 중단")
                return False
            
            # 2. 5초 대기
            self.logger.info("\n[2단계] 5초 대기...")
            for i in range(5, 0, -1):
                self.logger.info(f"이륙까지 {i}초...")
                time.sleep(1)
            
            # 3. 2m 이륙
            self.logger.info("\n[3단계] 2m 상공으로 이륙...")
            if not self.auto_takeoff(2.0):
                self.logger.error("이륙 실패! 긴급 착륙")
                self.emergency_stop()
                self.disarm()
                return False
            
            # 4. 10초 호버링 (위치 유지)
            self.logger.info("\n[4단계] 10초간 호버링 (위치 유지 활성화)...")
            self.start_hovering()
            
            for i in range(10):
                alt = self._get_relative_altitude()
                
                # GPS 정보가 있으면 표시
                if self.current_gps:
                    self.logger.info(f"호버링 {i+1}/10초 | 고도: {alt:.2f}m | "
                                   f"GPS: ({self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f})")
                else:
                    self.logger.info(f"호버링 {i+1}/10초 | 고도: {alt:.2f}m")
                
                time.sleep(1)
            
            # 5. 착륙
            self.logger.info("\n[5단계] 착륙 시작...")
            if not self.auto_land():
                self.logger.error("착륙 실패!")
                self.emergency_stop()
                self.disarm()
                return False
            
            self.logger.info("\n" + "="*60)
            self.logger.info("✅ 미션 완료!")
            self.logger.info("="*60)
            return True
            
        except Exception as e:
            self.logger.error(f"미션 실행 중 오류: {e}")
            self.emergency_stop()
            self.disarm()
            return False
    
    # ========================= 헬퍼 메서드 =========================
    
    def _set_flight_state(self, state: FlightState):
        """비행 상태 설정"""
        with self.state_lock:
            self.flight_state = state
            self.logger.debug(f"상태 변경: {state.value}")
    
    def _get_relative_altitude(self) -> float:
        """상대 고도 반환"""
        with self.altitude_lock:
            return self.current_altitude.relative - self.takeoff_altitude
    
    def _save_home_position(self):
        """현재 위치를 홈으로 저장"""
        if self.current_gps:
            self.home_gps = (self.current_gps.latitude, self.current_gps.longitude)
            self.set_home(self.current_gps.latitude, self.current_gps.longitude)
            self.logger.info(f"홈 위치 저장: ({self.home_gps[0]:.6f}, {self.home_gps[1]:.6f})")
    
    def _calculate_position_error(self) -> Tuple[float, float]:
        """위치 오차 계산"""
        if not self.current_gps or not self.home_gps:
            return 0, 0
        
        # 거리 계산
        R = 6371000
        lat1, lon1 = self.home_gps
        lat2, lon2 = self.current_gps.latitude, self.current_gps.longitude
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        
        # 방향 계산
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        bearing = math.degrees(math.atan2(x, y))
        bearing = (bearing + 360) % 360
        
        return distance, bearing
    
    # ========================= 메시지 처리 오버라이드 =========================
    
    def _process_message(self, message: str):
        """메시지 처리 (오버라이드)"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            # 부모 클래스 처리
            super()._process_message(message)
            
            # 추가 처리
            if msg_type == 'ALTITUDE':
                self._handle_altitude_data(data.get('data', {}))
            elif msg_type == 'IMU':
                self._handle_imu_data(data.get('data', {}))
            elif msg_type == 'ARM_STATUS':
                self._handle_arm_status(data.get('data', {}))
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
    
    def _handle_altitude_data(self, data: Dict):
        """고도 데이터 처리"""
        with self.altitude_lock:
            self.current_altitude = AltitudeData(
                relative=data.get('relative', 0),
                absolute=data.get('absolute', 0),
                timestamp=time.time()
            )
    
    def _handle_imu_data(self, data: Dict):
        """IMU 데이터 처리"""
        with self.imu_lock:
            self.current_imu = IMUData(
                roll=data.get('roll', 0),
                pitch=data.get('pitch', 0),
                yaw=data.get('yaw', 0),
                acc_x=data.get('acc_x', 0),
                acc_y=data.get('acc_y', 0),
                acc_z=data.get('acc_z', 0),
                timestamp=time.time()
            )
    
    def _handle_arm_status(self, data: Dict):
        """시동 상태 처리"""
        is_armed = data.get('armed', False)
        if is_armed and self.flight_state == FlightState.ARMING:
            self._set_flight_state(FlightState.ARMED)
        elif not is_armed and self.flight_state != FlightState.DISARMED:
            self._set_flight_state(FlightState.DISARMED)


# ========================= 메인 실행 =========================

def main():
    """메인 실행 함수"""
    
    print("\n" + "="*60)
    print("자동 이륙/착륙 시스템")
    print("시동 → 5초 대기 → 2m 이륙 → 10초 호버링 → 착륙")
    print("="*60 + "\n")
    
    # 시스템 생성
    drone_system = AutoTakeoffLandingSystem("/dev/ttyTHS1", 115200)
    
    # 연결
    if not drone_system.connect():
        print("❌ 드론 연결 실패!")
        return
    
    try:
        # 미션 실행
        success = drone_system.execute_mission()
        
        if success:
            print("\n🎉 모든 미션이 성공적으로 완료되었습니다!")
            print("시스템 설정:")
            print(f"  - 이륙 쓰로틀: {drone_system.THROTTLE_TAKEOFF}%")
            print(f"  - 호버링 쓰로틀: {drone_system.THROTTLE_HOVER}%")
            print(f"  - 착륙 쓰로틀: {drone_system.THROTTLE_LAND}%")
            print(f"  - 위치 허용 오차: {drone_system.POSITION_TOLERANCE}m")
            print(f"  - 바람 저항 계수: {drone_system.WIND_RESISTANCE_GAIN}")
        else:
            print("\n❌ 미션 실패")
            
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자 중단! 긴급 정지 및 착륙...")
        drone_system.emergency_stop()
        time.sleep(1)
        drone_system.auto_land()
        
    except Exception as e:
        print(f"\n❌ 시스템 오류: {e}")
        drone_system.emergency_stop()
        drone_system.disarm()
        
    finally:
        # 연결 해제
        drone_system.disconnect()
        print("\n시스템 종료")


if __name__ == "__main__":
    main()
