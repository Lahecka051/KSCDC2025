"""
mavlink_drone_control.py
ArduPilot H743v2 FC + Jetson Orin Nano
4가지 데이터 [vertical, horizontal, rotation, speed] 제어
MAVLink2 통신 (/dev/ttyTHS1:115200)
"""

import time
import threading
import math
import logging
from typing import List, Tuple, Optional, Union
from dataclasses import dataclass
from enum import Enum
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)

# ========================= 상수 정의 =========================

# 쓰로틀 설정
THROTTLE_TAKEOFF = 90   # 이륙 쓰로틀 (%)
THROTTLE_HOVER = 60     # 호버링 쓰로틀 (%)
THROTTLE_LAND = 50      # 착륙 쓰로틀 (%)

# GPS 관련
GPS_REACH_THRESHOLD = 3.0  # 도달 판정 거리 (m)
MAX_SPEED_GPS = 80         # GPS 이동 시 최대 속도 (%)

# ========================= 드론 제어 클래스 =========================

class DroneControl:
    """
    4가지 데이터 기반 드론 제어
    [vertical, horizontal, rotation, speed]
    """
    
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        """
        초기화
        
        Args:
            port: UART 포트
            baudrate: 통신 속도
        """
        self.port = port
        self.baudrate = baudrate
        self.vehicle = None
        
        # 현재 명령
        self.current_cmd = ["level", "hover", 0, 0]
        self.cmd_lock = threading.Lock()
        
        # GPS 관련
        self.current_gps = None
        self.target_gps = None
        self.home_gps = None
        self.gps_navigation_active = False
        
        # 스레드
        self.gps_thread = None
        self.navigation_thread = None
        self.is_running = False
        
        logger.info(f"드론 제어 시스템 초기화: {port}:{baudrate}")
    
    # ========================= 연결 관리 =========================
    
    def connect(self) -> bool:
        """FC 연결"""
        try:
            logger.info(f"FC 연결 시도...")
            
            self.vehicle = connect(
                self.port,
                baud=self.baudrate,
                wait_ready=True,
                timeout=30
            )
            
            logger.info("✓ FC 연결 성공!")
            logger.info(f"  펌웨어: {self.vehicle.version}")
            logger.info(f"  모드: {self.vehicle.mode.name}")
            logger.info(f"  GPS: 위성 {self.vehicle.gps_0.satellites_visible}개")
            
            # 스레드 시작
            self.is_running = True
            self.gps_thread = threading.Thread(target=self._gps_update_loop, daemon=True)
            self.navigation_thread = threading.Thread(target=self._navigation_loop, daemon=True)
            self.gps_thread.start()
            self.navigation_thread.start()
            
            # 홈 위치 저장
            self.home_gps = (
                self.vehicle.location.global_frame.lat,
                self.vehicle.location.global_frame.lon,
                self.vehicle.location.global_relative_frame.alt
            )
            logger.info(f"홈 위치: ({self.home_gps[0]:.6f}, {self.home_gps[1]:.6f})")
            
            return True
            
        except Exception as e:
            logger.error(f"FC 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.is_running = False
        self.gps_navigation_active = False
        
        if self.gps_thread:
            self.gps_thread.join(timeout=1)
        if self.navigation_thread:
            self.navigation_thread.join(timeout=1)
        
        if self.vehicle:
            self.vehicle.close()
        
        logger.info("FC 연결 해제")
    
    # ========================= 메인 제어 명령 =========================
    
    def set_command(self, L1: str, L2: str, L3: float, L4: float) -> bool:
        """
        4가지 데이터로 드론 제어
        
        Args:
            L1: vertical (up/level/down)
            L2: horizontal (forward/backward/left/right/forward_left/forward_right/backward_left/backward_right/hover)
            L3: rotation (0~359 degree, 시계방향)
            L4: speed/throttle (0~100%)
        
        Returns:
            성공 여부
        """
        # 입력 검증
        valid_L1 = ["up", "level", "down"]
        valid_L2 = [
            "forward", "backward", "left", "right",
            "forward_left", "forward_right",
            "backward_left", "backward_right", "hover"
        ]
        
        if L1 not in valid_L1:
            logger.error(f"잘못된 L1(vertical): {L1}")
            return False
        
        if L2 not in valid_L2:
            logger.error(f"잘못된 L2(horizontal): {L2}")
            return False
        
        # L3 정규화 (0~359)
        L3 = L3 % 360
        
        # L4 제한 (0~100)
        L4 = max(0, min(100, L4))
        
        # 명령 저장
        with self.cmd_lock:
            self.current_cmd = [L1, L2, L3, L4]
        
        logger.info(f"명령: [{L1}, {L2}, {L3:.0f}°, {L4:.0f}%]")
        
        # FC로 전송
        self._send_to_fc(L1, L2, L3, L4)
        
        return True
    
    def _send_to_fc(self, vertical: str, horizontal: str, rotation: float, speed: float):
        """FC로 MAVLink 명령 전송"""
        try:
            if not self.vehicle:
                return
            
            # GUIDED 모드 확인
            if self.vehicle.mode.name != "GUIDED":
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(0.5)
            
            # 특수 쓰로틀 처리
            actual_throttle = speed
            if vertical == "up" and speed == 0:
                actual_throttle = THROTTLE_TAKEOFF
            elif vertical == "level" and horizontal == "hover" and speed == 0:
                actual_throttle = THROTTLE_HOVER
            elif vertical == "down" and speed == 0:
                actual_throttle = THROTTLE_LAND
            
            # 속도 변환 (% → m/s)
            velocity = actual_throttle * 0.1  # 100% = 10m/s
            
            # 수직 속도 (NED 좌표계: 상승=음수, 하강=양수)
            vz = 0
            if vertical == "up":
                vz = -velocity * 0.5
            elif vertical == "down":
                vz = velocity * 0.5
            
            # 수평 속도 (Body Frame)
            vx, vy = 0, 0
            if horizontal == "forward":
                vx = velocity
            elif horizontal == "backward":
                vx = -velocity
            elif horizontal == "left":
                vy = -velocity
            elif horizontal == "right":
                vy = velocity
            elif horizontal == "forward_left":
                vx = velocity * 0.707
                vy = -velocity * 0.707
            elif horizontal == "forward_right":
                vx = velocity * 0.707
                vy = velocity * 0.707
            elif horizontal == "backward_left":
                vx = -velocity * 0.707
                vy = -velocity * 0.707
            elif horizontal == "backward_right":
                vx = -velocity * 0.707
                vy = velocity * 0.707
            
            # Yaw 회전 (rad/s)
            yaw_rate = 0
            if rotation > 0:
                yaw_rate = math.radians(rotation) / 5  # 5초에 회전 완료
            
            # MAVLink 메시지 생성
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Body Frame
                0b0000111111000111,  # type_mask (속도 제어)
                0, 0, 0,  # 위치
                vx, vy, vz,  # 속도
                0, 0, 0,  # 가속도
                0, yaw_rate  # yaw, yaw_rate
            )
            
            self.vehicle.send_mavlink(msg)
            
        except Exception as e:
            logger.error(f"FC 전송 오류: {e}")
    
    # ========================= GPS 이동 =========================
    
    def goto_gps(self, latitude: float, longitude: float, altitude: Optional[float] = None):
        """
        GPS 좌표로 이동
        
        Args:
            latitude: 위도
            longitude: 경도
            altitude: 고도 (None이면 현재 고도 유지)
        """
        if altitude is None:
            altitude = self.vehicle.location.global_relative_frame.alt
        
        self.target_gps = (latitude, longitude, altitude)
        self.gps_navigation_active = True
        
        logger.info(f"GPS 이동 시작: ({latitude:.6f}, {longitude:.6f}, {altitude:.1f}m)")
    
    def goto_home(self):
        """홈으로 복귀"""
        if self.home_gps:
            self.goto_gps(self.home_gps[0], self.home_gps[1], self.home_gps[2])
        else:
            logger.warning("홈 위치가 설정되지 않음")
    
    def stop_gps_navigation(self):
        """GPS 네비게이션 중지"""
        self.gps_navigation_active = False
        self.set_command("level", "hover", 0, THROTTLE_HOVER)
        logger.info("GPS 네비게이션 중지")
    
    # ========================= GPS 업데이트 루프 =========================
    
    def _gps_update_loop(self):
        """GPS 데이터 업데이트"""
        while self.is_running:
            try:
                if self.vehicle:
                    self.current_gps = (
                        self.vehicle.location.global_frame.lat,
                        self.vehicle.location.global_frame.lon,
                        self.vehicle.location.global_relative_frame.alt
                    )
                
                time.sleep(0.2)  # 5Hz
                
            except Exception as e:
                logger.error(f"GPS 업데이트 오류: {e}")
                time.sleep(1)
    
    def _navigation_loop(self):
        """GPS 네비게이션 루프"""
        while self.is_running:
            try:
                if self.gps_navigation_active and self.target_gps and self.current_gps:
                    self._navigate_to_target()
                
                time.sleep(0.5)  # 2Hz
                
            except Exception as e:
                logger.error(f"네비게이션 오류: {e}")
                time.sleep(1)
    
    def _navigate_to_target(self):
        """목표 GPS로 네비게이션"""
        # 현재 위치
        cur_lat, cur_lon, cur_alt = self.current_gps
        tgt_lat, tgt_lon, tgt_alt = self.target_gps
        
        # 거리 계산
        distance = self._calculate_distance(cur_lat, cur_lon, tgt_lat, tgt_lon)
        
        # 도착 확인
        if distance < GPS_REACH_THRESHOLD:
            logger.info(f"목표 도달! (거리: {distance:.1f}m)")
            self.gps_navigation_active = False
            self.set_command("level", "hover", 0, THROTTLE_HOVER)
            return
        
        # 방향 계산
        bearing = self._calculate_bearing(cur_lat, cur_lon, tgt_lat, tgt_lon)
        current_heading = self.vehicle.heading
        
        # 회전 각도 계산
        rotation_needed = bearing - current_heading
        if rotation_needed > 180:
            rotation_needed -= 360
        elif rotation_needed < -180:
            rotation_needed += 360
        
        # 방향 결정
        L2 = "forward"  # 기본 전진
        if abs(rotation_needed) < 30:
            L2 = "forward"
        elif abs(rotation_needed) < 60:
            L2 = "forward_right" if rotation_needed > 0 else "forward_left"
        elif abs(rotation_needed) < 120:
            L2 = "right" if rotation_needed > 0 else "left"
        else:
            L2 = "backward"
        
        # 고도 조절
        L1 = "level"
        alt_diff = tgt_alt - cur_alt
        if alt_diff > 2:
            L1 = "up"
        elif alt_diff < -2:
            L1 = "down"
        
        # 속도 (거리에 비례)
        L4 = min(MAX_SPEED_GPS, max(30, distance * 5))
        
        # 회전
        L3 = abs(rotation_needed) if abs(rotation_needed) > 10 else 0
        
        # 명령 전송
        self.set_command(L1, L2, L3, L4)
        
        logger.info(f"GPS Nav: 거리={distance:.1f}m, 방향={bearing:.0f}°, 회전={rotation_needed:.0f}°")
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """GPS 거리 계산 (m)"""
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def _calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """GPS 방위각 계산 (0~359°)"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)
        
        x = math.sin(dlambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360
    
    # ========================= 편의 메서드 =========================
    
    def takeoff(self, altitude: float = 2.0):
        """이륙"""
        logger.info(f"이륙 시작 ({altitude}m)")
        self.vehicle.simple_takeoff(altitude)
        
        while self.vehicle.location.global_relative_frame.alt < altitude * 0.95:
            self.set_command("up", "hover", 0, THROTTLE_TAKEOFF)
            time.sleep(0.5)
        
        logger.info("이륙 완료")
        self.hover()
    
    def land(self):
        """착륙"""
        logger.info("착륙 시작")
        
        while self.vehicle.location.global_relative_frame.alt > 0.2:
            self.set_command("down", "hover", 0, THROTTLE_LAND)
            time.sleep(0.5)
        
        self.vehicle.armed = False
        logger.info("착륙 완료")
    
    def hover(self):
        """호버링"""
        self.set_command("level", "hover", 0, THROTTLE_HOVER)
    
    def forward(self, speed: float = 50):
        """전진"""
        self.set_command("level", "forward", 0, speed)
    
    def backward(self, speed: float = 50):
        """후진"""
        self.set_command("level", "backward", 0, speed)
    
    def left(self, speed: float = 50):
        """좌측"""
        self.set_command("level", "left", 0, speed)
    
    def right(self, speed: float = 50):
        """우측"""
        self.set_command("level", "right", 0, speed)
    
    def rotate(self, degree: float):
        """회전"""
        self.set_command("level", "hover", degree, 0)
    
    def diagonal(self, direction: str, speed: float = 50):
        """대각선 이동"""
        self.set_command("level", direction, 0, speed)
    
    def emergency_stop(self):
        """긴급 정지"""
        logger.warning("긴급 정지!")
        self.gps_navigation_active = False
        self.set_command("level", "hover", 0, 0)
    
    # ========================= 상태 정보 =========================
    
    def get_status(self) -> dict:
        """현재 상태 반환"""
        return {
            "connected": self.vehicle is not None,
            "armed": self.vehicle.armed if self.vehicle else False,
            "mode": self.vehicle.mode.name if self.vehicle else "N/A",
            "gps": self.current_gps,
            "target": self.target_gps if self.gps_navigation_active else None,
            "altitude": self.vehicle.location.global_relative_frame.alt if self.vehicle else 0,
            "battery": self.vehicle.battery.voltage if self.vehicle else 0,
            "command": self.current_cmd
        }


# ========================= 메인 실행 =========================

def main():
    """메인 실행 예제"""
    
    print("\n" + "="*60)
    print("드론 제어 시스템")
    print("4가지 데이터: [vertical, horizontal, rotation, speed]")
    print("="*60 + "\n")
    
    # 드론 객체 생성
    drone = DroneControl('/dev/ttyTHS1', 115200)
    
    # FC 연결
    if not drone.connect():
        print("❌ FC 연결 실패!")
        return
    
    try:
        # 시동
        print("\n시동 걸기...")
        drone.vehicle.mode = VehicleMode("GUIDED")
        drone.vehicle.armed = True
        while not drone.vehicle.armed:
            time.sleep(1)
        print("✓ 시동 완료")
        
        # 이륙 (2m)
        print("\n이륙 (2m)...")
        drone.takeoff(2.0)
        
        # 호버링
        print("호버링...")
        drone.hover()
        time.sleep(3)
      
        # 착륙
        print("\n착륙...")
        drone.land()
        
    except KeyboardInterrupt:
        print("\n\n긴급 정지!")
        drone.emergency_stop()
        drone.vehicle.mode = VehicleMode("LAND")
        
    except Exception as e:
        print(f"\n오류: {e}")
        drone.emergency_stop()
        
    finally:
        drone.disconnect()
        print("\n프로그램 종료")


if __name__ == "__main__":
    main()
