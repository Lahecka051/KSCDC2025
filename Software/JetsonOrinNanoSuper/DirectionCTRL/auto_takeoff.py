"""
auto_takeoff.py
드론 자동 시동 및 이륙 모듈 (위치 유지 기능 포함)
시동 재시도 + 2m 상승 + 호버링 + 위치 보정
"""

import serial
import json
import threading
import time
import logging
import math
from typing import Optional, Dict, Any, Tuple
from enum import Enum
from dataclasses import dataclass

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

# ========================= 상태 정의 =========================

class DroneState(Enum):
    """드론 상태"""
    DISARMED = "disarmed"          # 시동 꺼짐
    ARMING = "arming"              # 시동 중
    ARMED = "armed"                # 시동 걸림
    TAKING_OFF = "taking_off"      # 이륙 중
    HOVERING = "hovering"          # 호버링
    FLYING = "flying"              # 비행 중
    LANDING = "landing"            # 착륙 중
    ERROR = "error"                # 오류

@dataclass
class AltitudeData:
    """고도 데이터"""
    relative: float    # 상대 고도 (이륙 지점 기준)
    absolute: float    # 절대 고도 (해수면 기준)
    barometric: float  # 기압계 고도
    timestamp: float

@dataclass
class PositionData:
    """위치 데이터 (GPS 또는 옵티컬 플로우)"""
    latitude: float    # 위도
    longitude: float   # 경도
    x_offset: float   # X축 오프셋 (m) - 옵티컬 플로우
    y_offset: float   # Y축 오프셋 (m) - 옵티컬 플로우
    heading: float    # 방향 (0-359도)
    timestamp: float

@dataclass
class IMUData:
    """IMU 센서 데이터"""
    roll: float       # 롤 (좌우 기울기)
    pitch: float      # 피치 (전후 기울기)
    yaw: float        # 요 (회전)
    acc_x: float      # X축 가속도
    acc_y: float      # Y축 가속도
    acc_z: float      # Z축 가속도
    timestamp: float

# ========================= 자동 이륙 드론 클래스 =========================

class AutoTakeoffDrone:
    """자동 시동 및 이륙 드론 시스템 (위치 유지 기능 포함)"""
    
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200):
        """
        초기화
        
        Args:
            port: FC 통신 포트
            baudrate: 통신 속도
        """
        self.logger = logging.getLogger(__name__)
        
        # 시리얼 통신
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        
        # 상태 관리
        self.state = DroneState.DISARMED
        self.state_lock = threading.Lock()
        
        # 고도 데이터
        self.current_altitude = AltitudeData(0, 0, 0, time.time())
        self.altitude_lock = threading.Lock()
        self.takeoff_altitude = 0  # 이륙 시점 고도
        
        # 위치 데이터
        self.current_position = PositionData(0, 0, 0, 0, 0, time.time())
        self.home_position = None  # 이륙 지점 위치
        self.position_lock = threading.Lock()
        
        # IMU 데이터
        self.current_imu = IMUData(0, 0, 0, 0, 0, 0, time.time())
        self.imu_lock = threading.Lock()
        
        # 시동 관련
        self.arm_attempts = 0
        self.max_arm_attempts = 10
        self.arm_retry_delay = 2.0  # 초
        
        # 쓰로틀 설정 (수정됨)
        self.THROTTLE_TAKEOFF = 90.0   # 상승 쓰로틀 (%)
        self.THROTTLE_HOVER = 60.0     # 호버링 쓰로틀 (%)
        self.THROTTLE_DESCEND = 50.0   # 하강 쓰로틀 (%)
        
        # 이륙 파라미터
        self.TARGET_ALTITUDE = 2.0  # 목표 고도 (m)
        self.ALTITUDE_TOLERANCE = 0.2  # 고도 허용 오차 (m)
        
        # 위치 유지 파라미터
        self.POSITION_TOLERANCE = 0.3  # 위치 허용 오차 (m)
        self.MAX_POSITION_ERROR = 1.0  # 최대 위치 오차 (m)
        self.POSITION_CORRECTION_SPEED = 30.0  # 위치 보정 속도 (%)
        
        # 자세 제어 파라미터
        self.MAX_TILT_ANGLE = 10.0  # 최대 기울기 각도 (도)
        self.WIND_RESISTANCE_GAIN = 1.5  # 바람 저항 보정 계수
        
        # 안전 파라미터
        self.MAX_TAKEOFF_TIME = 30.0  # 최대 이륙 시간 (초)
        self.MIN_ARM_VOLTAGE = 11.0   # 최소 시동 전압
        
        # 스레드 관리
        self.is_running = False
        self.rx_thread = None
        self.monitor_thread = None
        self.position_hold_thread = None
        
        # 위치 유지 활성화 플래그
        self.position_hold_enabled = False
        
        # 콜백
        self.arm_success_callback = None
        self.takeoff_complete_callback = None
        self.error_callback = None
        
    # ========================= 연결 관리 =========================
    
    def connect(self) -> bool:
        """FC 연결"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            self.is_running = True
            
            # 스레드 시작
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            
            self.position_hold_thread = threading.Thread(target=self._position_hold_loop, daemon=True)
            self.position_hold_thread.start()
            
            self.logger.info(f"드론 연결 성공: {self.port}@{self.baudrate}")
            return True
            
        except Exception as e:
            self.logger.error(f"연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.is_running = False
        self.position_hold_enabled = False
        
        # 안전을 위해 시동 끄기
        if self.state != DroneState.DISARMED:
            self.disarm()
        
        if self.rx_thread:
            self.rx_thread.join(timeout=1)
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1)
        if self.position_hold_thread:
            self.position_hold_thread.join(timeout=1)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.logger.info("드론 연결 해제")
    
    # ========================= 메인 기능: 자동 시동 및 이륙 =========================
    
    def auto_arm_and_takeoff(self) -> bool:
        """
        자동 시동 및 이륙
        - 시동 실패시 자동 재시도
        - 시동 성공시 2m 상승 후 호버링
        - 위치 유지 기능 자동 활성화
        
        Returns:
            성공 여부
        """
        self.logger.info("="*50)
        self.logger.info("자동 시동 및 이륙 시작")
        self.logger.info("="*50)
        
        # 현재 위치를 홈으로 저장
        self._save_home_position()
        
        # 1단계: 시동 걸기 (실패시 재시도)
        if not self._arm_with_retry():
            self.logger.error("시동 걸기 최종 실패")
            self._set_state(DroneState.ERROR)
            if self.error_callback:
                self.error_callback("ARM_FAILED")
            return False
        
        # 2단계: 이륙 (2m 상승)
        if not self._takeoff_to_altitude():
            self.logger.error("이륙 실패")
            self._set_state(DroneState.ERROR)
            self.disarm()  # 안전을 위해 시동 끄기
            if self.error_callback:
                self.error_callback("TAKEOFF_FAILED")
            return False
        
        # 3단계: 호버링 상태 유지 + 위치 유지 활성화
        self._start_hovering()
        self.position_hold_enabled = True  # 위치 유지 기능 활성화
        
        self.logger.info("자동 이륙 완료! 호버링 중 (위치 유지 모드 ON)")
        if self.takeoff_complete_callback:
            self.takeoff_complete_callback()
        
        return True
    
    # ========================= 시동 관리 =========================
    
    def _arm_with_retry(self) -> bool:
        """
        시동 걸기 (재시도 포함)
        
        Returns:
            성공 여부
        """
        self.arm_attempts = 0
        
        while self.arm_attempts < self.max_arm_attempts:
            self.arm_attempts += 1
            self.logger.info(f"시동 시도 {self.arm_attempts}/{self.max_arm_attempts}")
            
            # 시동 명령 전송
            if self._send_arm_command():
                # 응답 대기
                time.sleep(1)
                
                # 상태 확인
                if self._check_armed_status():
                    self.logger.info(f"✓ 시동 성공! (시도: {self.arm_attempts})")
                    self._set_state(DroneState.ARMED)
                    
                    if self.arm_success_callback:
                        self.arm_success_callback()
                    
                    return True
                else:
                    self.logger.warning(f"시동 실패 - {self.arm_retry_delay}초 후 재시도...")
            
            time.sleep(self.arm_retry_delay)
        
        self.logger.error(f"시동 실패 (최대 시도 횟수 초과)")
        return False
    
    def _send_arm_command(self) -> bool:
        """FC로 시동 명령 전송"""
        try:
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
                self.logger.debug("ARM 명령 전송")
                return True
                
        except Exception as e:
            self.logger.error(f"ARM 명령 전송 오류: {e}")
        
        return False
    
    def _check_armed_status(self) -> bool:
        """시동 상태 확인"""
        try:
            command = {
                'type': 'STATUS',
                'data': {
                    'request': 'arm_status'
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                
                time.sleep(0.5)
                
                with self.state_lock:
                    return self.state == DroneState.ARMED
                    
        except Exception as e:
            self.logger.error(f"상태 확인 오류: {e}")
        
        return False
    
    def disarm(self) -> bool:
        """시동 끄기"""
        try:
            self.position_hold_enabled = False  # 위치 유지 비활성화
            
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
                
                self._set_state(DroneState.DISARMED)
                self.logger.info("시동 끄기 완료")
                return True
                
        except Exception as e:
            self.logger.error(f"DISARM 오류: {e}")
        
        return False
    
    # ========================= 이륙 제어 =========================
    
    def _takeoff_to_altitude(self) -> bool:
        """
        목표 고도까지 이륙
        
        Returns:
            성공 여부
        """
        self.logger.info(f"이륙 시작 - 목표 고도: {self.TARGET_ALTITUDE}m (쓰로틀: {self.THROTTLE_TAKEOFF}%)")
        self._set_state(DroneState.TAKING_OFF)
        
        # 현재 고도 저장 (이륙 기준점)
        with self.altitude_lock:
            self.takeoff_altitude = self.current_altitude.relative
        
        start_time = time.time()
        
        # 상승 명령 전송 (90% 쓰로틀)
        self._send_control_command("up", "hover", 0, self.THROTTLE_TAKEOFF)
        
        while True:
            # 현재 상대 고도
            current_rel_altitude = self._get_relative_altitude()
            
            # 목표 도달 확인
            if current_rel_altitude >= (self.TARGET_ALTITUDE - self.ALTITUDE_TOLERANCE):
                self.logger.info(f"✓ 목표 고도 도달: {current_rel_altitude:.2f}m")
                return True
            
            # 타임아웃 확인
            elapsed = time.time() - start_time
            if elapsed > self.MAX_TAKEOFF_TIME:
                self.logger.error(f"이륙 타임아웃 ({self.MAX_TAKEOFF_TIME}초 초과)")
                return False
            
            # 상태 로그
            if int(elapsed) % 2 == 0:  # 2초마다 로그
                self.logger.info(f"상승 중... 현재 고도: {current_rel_altitude:.2f}m / 목표: {self.TARGET_ALTITUDE}m")
            
            time.sleep(0.1)
    
    def _start_hovering(self):
        """호버링 시작 (60% 쓰로틀)"""
        self._set_state(DroneState.HOVERING)
        
        try:
            self._send_control_command("level", "hover", 0, self.THROTTLE_HOVER)
            self.logger.info(f"호버링 모드 활성화 (쓰로틀: {self.THROTTLE_HOVER}%)")
                
        except Exception as e:
            self.logger.error(f"호버링 명령 오류: {e}")
    
    # ========================= 위치 유지 기능 =========================
    
    def _position_hold_loop(self):
        """위치 유지 제어 루프"""
        while self.is_running:
            try:
                if self.position_hold_enabled and self.state == DroneState.HOVERING:
                    self._maintain_position()
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.logger.error(f"위치 유지 루프 오류: {e}")
                time.sleep(0.5)
    
    def _maintain_position(self):
        """위치 유지 (바람 보정)"""
        if not self.home_position:
            return
        
        with self.position_lock:
            current_pos = self.current_position
        
        with self.imu_lock:
            current_imu = self.current_imu
        
        # GPS 기반 위치 오차 계산
        if current_pos.latitude != 0 and current_pos.longitude != 0:
            distance, bearing = self._calculate_position_error(
                self.home_position[0], self.home_position[1],
                current_pos.latitude, current_pos.longitude
            )
            
            # 위치 오차가 허용치를 벗어난 경우
            if distance > self.POSITION_TOLERANCE:
                self._correct_position_gps(distance, bearing)
        
        # 옵티컬 플로우 기반 위치 보정
        elif current_pos.x_offset != 0 or current_pos.y_offset != 0:
            offset_distance = math.sqrt(current_pos.x_offset**2 + current_pos.y_offset**2)
            
            if offset_distance > self.POSITION_TOLERANCE:
                self._correct_position_optical(current_pos.x_offset, current_pos.y_offset)
        
        # IMU 기반 자세 보정 (바람에 의한 기울기 감지)
        if abs(current_imu.roll) > self.MAX_TILT_ANGLE or abs(current_imu.pitch) > self.MAX_TILT_ANGLE:
            self._correct_attitude(current_imu.roll, current_imu.pitch)
    
    def _correct_position_gps(self, distance: float, bearing: float):
        """GPS 기반 위치 보정"""
        self.logger.debug(f"위치 보정: 거리={distance:.2f}m, 방향={bearing:.0f}°")
        
        # 보정 방향 결정
        heading_diff = bearing - self.current_position.heading
        if heading_diff > 180:
            heading_diff -= 360
        elif heading_diff < -180:
            heading_diff += 360
        
        # 이동 방향 결정
        if abs(heading_diff) < 45:
            horizontal = "backward"  # 반대 방향으로 이동
        elif abs(heading_diff) < 135:
            if heading_diff > 0:
                horizontal = "left"
            else:
                horizontal = "right"
        else:
            horizontal = "forward"
        
        # 거리에 비례한 속도 설정
        correction_speed = min(self.POSITION_CORRECTION_SPEED, distance * 20)
        
        # 보정 명령 전송
        self._send_control_command("level", horizontal, 0, correction_speed)
        time.sleep(0.2)  # 짧은 보정
        
        # 다시 호버링
        self._start_hovering()
    
    def _correct_position_optical(self, x_offset: float, y_offset: float):
        """옵티컬 플로우 기반 위치 보정"""
        self.logger.debug(f"옵티컬 위치 보정: X={x_offset:.2f}m, Y={y_offset:.2f}m")
        
        # X축 보정 (좌우)
        if abs(x_offset) > self.POSITION_TOLERANCE:
            if x_offset > 0:
                self._send_control_command("level", "left", 0, self.POSITION_CORRECTION_SPEED)
            else:
                self._send_control_command("level", "right", 0, self.POSITION_CORRECTION_SPEED)
            time.sleep(0.1)
        
        # Y축 보정 (전후)
        if abs(y_offset) > self.POSITION_TOLERANCE:
            if y_offset > 0:
                self._send_control_command("level", "backward", 0, self.POSITION_CORRECTION_SPEED)
            else:
                self._send_control_command("level", "forward", 0, self.POSITION_CORRECTION_SPEED)
            time.sleep(0.1)
        
        # 다시 호버링
        self._start_hovering()
    
    def _correct_attitude(self, roll: float, pitch: float):
        """자세 보정 (바람 저항)"""
        self.logger.debug(f"자세 보정: Roll={roll:.1f}°, Pitch={pitch:.1f}°")
        
        # 롤 보정 (좌우 기울기)
        if abs(roll) > self.MAX_TILT_ANGLE:
            correction_speed = abs(roll) * self.WIND_RESISTANCE_GAIN
            if roll > 0:  # 우측으로 기울어짐
                self._send_control_command("level", "left", 0, correction_speed)
            else:  # 좌측으로 기울어짐
                self._send_control_command("level", "right", 0, correction_speed)
            time.sleep(0.1)
        
        # 피치 보정 (전후 기울기)
        if abs(pitch) > self.MAX_TILT_ANGLE:
            correction_speed = abs(pitch) * self.WIND_RESISTANCE_GAIN
            if pitch > 0:  # 앞으로 기울어짐
                self._send_control_command("level", "backward", 0, correction_speed)
            else:  # 뒤로 기울어짐
                self._send_control_command("level", "forward", 0, correction_speed)
            time.sleep(0.1)
        
        # 다시 호버링
        self._start_hovering()
    
    def _save_home_position(self):
        """현재 위치를 홈으로 저장"""
        with self.position_lock:
            if self.current_position.latitude != 0:
                self.home_position = (
                    self.current_position.latitude,
                    self.current_position.longitude
                )
                self.logger.info(f"홈 위치 저장: ({self.home_position[0]:.6f}, {self.home_position[1]:.6f})")
            else:
                # GPS가 없을 경우 옵티컬 플로우 원점으로 설정
                self.home_position = (0, 0)
                self.logger.info("홈 위치 저장: 옵티컬 플로우 원점")
    
    # ========================= 데이터 처리 =========================
    
    def _rx_loop(self):
        """수신 루프"""
        buffer = ""
        
        while self.is_running:
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line:
                            self._process_message(line.strip())
                
                time.sleep(0.001)
                
            except Exception as e:
                self.logger.error(f"수신 오류: {e}")
                time.sleep(0.1)
    
    def _process_message(self, message: str):
        """메시지 처리"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'ARM_STATUS':
                self._handle_arm_status(data.get('data', {}))
            elif msg_type == 'ALTITUDE':
                self._handle_altitude_data(data.get('data', {}))
            elif msg_type == 'GPS':
                self._handle_gps_data(data.get('data', {}))
            elif msg_type == 'OPTICAL_FLOW':
                self._handle_optical_flow_data(data.get('data', {}))
            elif msg_type == 'IMU':
                self._handle_imu_data(data.get('data', {}))
            elif msg_type == 'ERROR':
                self._handle_error(data.get('data', {}))
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
    
    def _handle_arm_status(self, data: Dict):
        """시동 상태 처리"""
        is_armed = data.get('armed', False)
        
        with self.state_lock:
            if is_armed and self.state == DroneState.DISARMED:
                self.state = DroneState.ARMED
                self.logger.info("시동 상태: ARMED")
            elif not is_armed and self.state != DroneState.DISARMED:
                self.state = DroneState.DISARMED
                self.logger.info("시동 상태: DISARMED")
    
    def _handle_altitude_data(self, data: Dict):
        """고도 데이터 처리"""
        with self.altitude_lock:
            self.current_altitude = AltitudeData(
                relative=data.get('relative', 0),
                absolute=data.get('absolute', 0),
                barometric=data.get('barometric', 0),
                timestamp=time.time()
            )
    
    def _handle_gps_data(self, data: Dict):
        """GPS 데이터 처리"""
        with self.position_lock:
            self.current_position.latitude = data.get('lat', 0)
            self.current_position.longitude = data.get('lon', 0)
            self.current_position.heading = data.get('heading', 0)
            self.current_position.timestamp = time.time()
    
    def _handle_optical_flow_data(self, data: Dict):
        """옵티컬 플로우 데이터 처리"""
        with self.position_lock:
            self.current_position.x_offset = data.get('x', 0)
            self.current_position.y_offset = data.get('y', 0)
            self.current_position.timestamp = time.time()
    
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
    
    def _handle_error(self, data: Dict):
        """오류 처리"""
        error_code = data.get('code', 'UNKNOWN')
        error_msg = data.get('message', '')
        
        self.logger.error(f"FC 오류: [{error_code}] {error_msg}")
        
        if self.error_callback:
            self.error_callback(error_code)
    
    # ========================= 모니터링 =========================
    
    def _monitor_loop(self):
        """시스템 모니터링 루프"""
        while self.is_running:
            try:
                # 고도 모니터링 (호버링 중)
                if self.state == DroneState.HOVERING:
                    current_alt = self._get_relative_altitude()
                    
                    # 고도 유지 확인
                    if abs(current_alt - self.TARGET_ALTITUDE) > self.ALTITUDE_TOLERANCE * 2:
                        self.logger.warning(f"고도 편차: {current_alt:.2f}m (목표: {self.TARGET_ALTITUDE}m)")
                        self._adjust_altitude(current_alt)
                
                time.sleep(0.5)
                
            except Exception as e:
                self.logger.error(f"모니터링 오류: {e}")
                time.sleep(1)
    
    def _adjust_altitude(self, current_alt: float):
        """고도 보정"""
        if current_alt < self.TARGET_ALTITUDE - self.ALTITUDE_TOLERANCE:
            # 상승 필요 (60% 쓰로틀로 상승)
            self._send_control_command("up", "hover", 0, self.THROTTLE_HOVER)
            time.sleep(0.3)
            self._start_hovering()
        elif current_alt > self.TARGET_ALTITUDE + self.ALTITUDE_TOLERANCE:
            # 하강 필요 (50% 쓰로틀로 하강)
            self._send_control_command("down", "hover", 0, self.THROTTLE_DESCEND)
            time.sleep(0.3)
            self._start_hovering()
    
    # ========================= 제어 명령 =========================
    
    def _send_control_command(self, vertical: str, horizontal: str, rotation: float, speed: float):
        """제어 명령 전송"""
        try:
            command = {
                'type': 'CONTROL',
                'data': {
                    'vertical': vertical,
                    'horizontal': horizontal,
                    'rotation': rotation,
                    'speed': speed,
                    'timestamp': time.time()
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                self.logger.debug(f"제어 명령: [{vertical}, {horizontal}, {rotation:.0f}°, {speed:.0f}%]")
                
        except Exception as e:
            self.logger.error(f"제어 명령 오류: {e}")
    
    # ========================= 유틸리티 =========================
    
    def _set_state(self, state: DroneState):
        """상태 설정"""
        with self.state_lock:
            self.state = state
            self.logger.debug(f"상태 변경: {state.value}")
    
    def get_state(self) -> DroneState:
        """현재 상태 반환"""
        with self.state_lock:
            return self.state
    
    def _get_relative_altitude(self) -> float:
        """상대 고도 반환"""
        with self.altitude_lock:
            return self.current_altitude.relative - self.takeoff_altitude
    
    def get_altitude(self) -> AltitudeData:
        """고도 데이터 반환"""
        with self.altitude_lock:
            return self.current_altitude
    
    def get_position(self) -> PositionData:
        """위치 데이터 반환"""
        with self.position_lock:
            return self.current_position
    
    def _calculate_position_error(self, lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
        """
        GPS 좌표 간 거리와 방향 계산
        
        Returns:
            (거리(m), 방향(도))
        """
        R = 6371000  # 지구 반경
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        # 거리 계산
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
    
    def toggle_position_hold(self, enabled: bool):
        """위치 유지 기능 토글"""
        self.position_hold_enabled = enabled
        status = "ON" if enabled else "OFF"
        self.logger.info(f"위치 유지 모드: {status}")
    
    # ========================= 콜백 설정 =========================
    
    def set_arm_callback(self, callback):
        """시동 성공 콜백"""
        self.arm_success_callback = callback
    
    def set_takeoff_callback(self, callback):
        """이륙 완료 콜백"""
        self.takeoff_complete_callback = callback
    
    def set_error_callback(self, callback):
        """오류 콜백"""
        self.error_callback = callback


# ========================= 메인 실행 =========================

def main():
    """메인 실행 함수"""
    
    print("="*60)
    print("드론 자동 시동 및 이륙 시스템")
    print("위치 유지 기능 포함")
    print("="*60)
    
    # 드론 생성
    drone = AutoTakeoffDrone("/dev/ttyTHS1", 115200)
    
    # 콜백 설정
    def on_arm_success():
        print("\n🚁 시동 걸림! 이륙 준비 완료")
    
    def on_takeoff_complete():
        print("\n✈️ 이륙 완료! 2m 고도에서 호버링 중...")
        print("📍 위치 유지 모드 활성화")
    
    def on_error(error_code):
        print(f"\n❌ 오류 발생: {error_code}")
    
    drone.set_arm_callback(on_arm_success)
    drone.set_takeoff_callback(on_takeoff_complete)
    drone.set_error_callback(on_error)
    
    # 연결
    if not drone.connect():
        print("드론 연결 실패!")
        return
    
    try:
        # 자동 시동 및 이륙 실행
        success = drone.auto_arm_and_takeoff()
        
        if success:
            print("\n✅ 자동 이륙 성공!")
            print("쓰로틀 설정:")
            print(f"  - 상승: {drone.THROTTLE_TAKEOFF}%")
            print(f"  - 호버링: {drone.THROTTLE_HOVER}%")
            print(f"  - 하강: {drone.THROTTLE_DESCEND}%")
            
            # 호버링 유지 (30초)
            print("\n30초간 호버링 유지 (위치 보정 중)...")
            for i in range(30):
                alt_data = drone.get_altitude()
                pos_data = drone.get_position()
                rel_alt = drone._get_relative_altitude()
                
                print(f"\r고도: {rel_alt:.2f}m | ", end="")
                
                if pos_data.latitude != 0:
                    print(f"GPS: ({pos_data.latitude:.6f}, {pos_data.longitude:.6f}) | ", end="")
                else:
                    print(f"옵티컬: X={pos_data.x_offset:.2f}m, Y={pos_data.y_offset:.2f}m | ", end="")
                
                print(f"상태: {drone.get_state().value}", end="")
                time.sleep(1)
            
            print("\n\n착륙 시작...")
            drone.disarm()
            
        else:
            print("\n❌ 자동 이륙 실패")
            
    except KeyboardInterrupt:
        print("\n\n긴급 중단!")
        drone.disarm()
    
    finally:
        drone.disconnect()
        print("\n프로그램 종료")


if __name__ == "__main__":
    main()
