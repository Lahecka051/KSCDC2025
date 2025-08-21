"""
drone_control.py
드론 제어 핵심 모듈
ArduPilot H743v2 FC + Jetson Orin Nano
MAVLink2 통신 (/dev/ttyTHS1:115200)
"""

import time
import threading
import math
import logging
from typing import List, Tuple, Optional, Union, Dict, Any
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

# 안전 관련
MIN_BATTERY_VOLTAGE = 11.0  # 최소 배터리 전압 (V)
MAX_ALTITUDE = 50.0         # 최대 고도 (m)
GPS_MIN_SATELLITES = 6      # 최소 GPS 위성 수

# ========================= 데이터 클래스 =========================

@dataclass
class DroneStatus:
    """드론 상태"""
    connected: bool
    armed: bool
    mode: str
    latitude: float
    longitude: float
    altitude: float
    heading: float
    battery_voltage: float
    battery_level: float
    gps_satellites: int
    current_command: List
    is_flying: bool
    home_location: Tuple[float, float, float]

@dataclass
class FlightStats:
    """비행 통계"""
    total_distance: float = 0.0
    max_altitude: float = 0.0
    flight_time: float = 0.0
    waypoints_visited: int = 0

# ========================= 드론 제어 클래스 =========================

class DroneControl:
    """
    드론 제어 핵심 클래스
    4가지 데이터 기반 제어: [vertical, horizontal, rotation, speed]
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
        
        # 비행 통계
        self.flight_stats = FlightStats()
        self.flight_start_time = None
        
        # 스레드
        self.gps_thread = None
        self.navigation_thread = None
        self.monitor_thread = None
        self.is_running = False
        
        # 콜백
        self.on_gps_reached = None
        self.on_battery_low = None
        self.on_mode_changed = None
        
        logger.info(f"드론 제어 시스템 초기화: {port}:{baudrate}")
    
    # ========================= 연결 관리 =========================
    
    def connect(self, timeout: int = 30) -> bool:
        """
        FC 연결
        
        Args:
            timeout: 연결 타임아웃 (초)
        
        Returns:
            연결 성공 여부
        """
        try:
            logger.info(f"FC 연결 시도... (타임아웃: {timeout}초)")
            
            self.vehicle = connect(
                self.port,
                baud=self.baudrate,
                wait_ready=True,
                timeout=timeout
            )
            
            # 연결 정보 출력
            self._print_connection_info()
            
            # 초기 설정
            self._initialize_settings()
            
            # 스레드 시작
            self._start_threads()
            
            # 홈 위치 저장
            self._save_home_location()
            
            return True
            
        except Exception as e:
            logger.error(f"FC 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        logger.info("연결 해제 시작...")
        
        # 스레드 종료
        self.is_running = False
        self.gps_navigation_active = False
        
        if self.gps_thread:
            self.gps_thread.join(timeout=1)
        if self.navigation_thread:
            self.navigation_thread.join(timeout=1)
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1)
        
        # 차량 연결 해제
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
        
        logger.info("연결 해제 완료")
    
    def _print_connection_info(self):
        """연결 정보 출력"""
        logger.info("="*50)
        logger.info("✓ FC 연결 성공!")
        logger.info(f"  펌웨어: {self.vehicle.version}")
        logger.info(f"  기체: {self.vehicle.system_status.state}")
        logger.info(f"  모드: {self.vehicle.mode.name}")
        logger.info(f"  GPS: 위성 {self.vehicle.gps_0.satellites_visible}개 (Fix: {self.vehicle.gps_0.fix_type})")
        logger.info(f"  배터리: {self.vehicle.battery.voltage:.1f}V ({self.vehicle.battery.level}%)")
        logger.info("="*50)
    
    def _initialize_settings(self):
        """초기 설정"""
        # 이벤트 리스너 등록
        @self.vehicle.on_attribute('mode')
        def mode_listener(self, attr_name, value):
            logger.info(f"모드 변경: {value.name}")
            if self.on_mode_changed:
                self.on_mode_changed(value.name)
        
        @self.vehicle.on_attribute('battery')
        def battery_listener(self, attr_name, value):
            if value.voltage < MIN_BATTERY_VOLTAGE:
                logger.warning(f"배터리 부족: {value.voltage:.1f}V")
                if self.on_battery_low:
                    self.on_battery_low(value.voltage)
    
    def _start_threads(self):
        """스레드 시작"""
        self.is_running = True
        
        self.gps_thread = threading.Thread(target=self._gps_update_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self._navigation_loop, daemon=True)
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        
        self.gps_thread.start()
        self.navigation_thread.start()
        self.monitor_thread.start()
        
        logger.info("백그라운드 스레드 시작")
    
    def _save_home_location(self):
        """홈 위치 저장"""
        if self.vehicle.location.global_frame:
            self.home_gps = (
                self.vehicle.location.global_frame.lat,
                self.vehicle.location.global_frame.lon,
                self.vehicle.location.global_relative_frame.alt
            )
            logger.info(f"홈 위치: ({self.home_gps[0]:.6f}, {self.home_gps[1]:.6f}, {self.home_gps[2]:.1f}m)")
    
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
        # 연결 확인
        if not self.vehicle:
            logger.error("드론이 연결되지 않음")
            return False
        
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
        
        logger.debug(f"명령: [{L1}, {L2}, {L3:.0f}°, {L4:.0f}%]")
        
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
            
            # 수직 속도 (NED 좌표계)
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
            
            # Yaw 회전
            yaw_rate = 0
            if rotation > 0:
                yaw_rate = math.radians(rotation) / 5
            
            # MAVLink 메시지
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, yaw_rate
            )
            
            self.vehicle.send_mavlink(msg)
            
        except Exception as e:
            logger.error(f"FC 전송 오류: {e}")
    
    # ========================= GPS 이동 =========================
    
    def goto_gps(self, latitude: float, longitude: float, altitude: Optional[float] = None) -> bool:
        """
        GPS 좌표로 이동
        
        Args:
            latitude: 위도
            longitude: 경도
            altitude: 고도 (None이면 현재 고도 유지)
        
        Returns:
            명령 성공 여부
        """
        if not self.vehicle:
            logger.error("드론이 연결되지 않음")
            return False
        
        if altitude is None:
            altitude = self.vehicle.location.global_relative_frame.alt
        
        # 고도 제한
        altitude = min(altitude, MAX_ALTITUDE)
        
        self.target_gps = (latitude, longitude, altitude)
        self.gps_navigation_active = True
        
        logger.info(f"GPS 이동 시작: ({latitude:.6f}, {longitude:.6f}, {altitude:.1f}m)")
        return True
    
    def goto_home(self) -> bool:
        """홈으로 복귀"""
        if self.home_gps:
            return self.goto_gps(self.home_gps[0], self.home_gps[1], self.home_gps[2])
        else:
            logger.warning("홈 위치가 설정되지 않음")
            return False
    
    def stop_gps_navigation(self):
        """GPS 네비게이션 중지"""
        self.gps_navigation_active = False
        self.hover()
        logger.info("GPS 네비게이션 중지")
    
    # ========================= 기본 동작 =========================
    
    def arm(self, timeout: int = 30) -> bool:
        """
        시동 걸기
        
        Args:
            timeout: 타임아웃 (초)
        
        Returns:
            성공 여부
        """
        if not self.vehicle:
            return False
        
        logger.info("시동 절차 시작...")
        
        # 시동 가능 상태 확인
        while not self.vehicle.is_armable:
            logger.info("시동 가능 상태 대기 중...")
            time.sleep(1)
        
        # GUIDED 모드 설정
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode.name != "GUIDED":
            time.sleep(0.5)
        
        # 시동
        self.vehicle.armed = True
        
        start_time = time.time()
        while not self.vehicle.armed:
            if time.time() - start_time > timeout:
                logger.error("시동 타임아웃")
                return False
            time.sleep(1)
        
        logger.info("✓ 시동 성공")
        self.flight_start_time = time.time()
        return True
    
    def disarm(self) -> bool:
        """시동 끄기"""
        if not self.vehicle:
            return False
        
        self.vehicle.armed = False
        
        while self.vehicle.armed:
            time.sleep(1)
        
        logger.info("시동 꺼짐")
        
        # 비행 시간 계산
        if self.flight_start_time:
            self.flight_stats.flight_time = time.time() - self.flight_start_time
            logger.info(f"총 비행 시간: {self.flight_stats.flight_time:.1f}초")
        
        return True
    
    def takeoff(self, altitude: float = 2.0, wait: bool = True) -> bool:
        """
        이륙
        
        Args:
            altitude: 목표 고도 (m)
            wait: 이륙 완료 대기 여부
        
        Returns:
            성공 여부
        """
        if not self.vehicle or not self.vehicle.armed:
            logger.error("시동이 걸려있지 않음")
            return False
        
        logger.info(f"이륙 시작 ({altitude}m)")
        self.vehicle.simple_takeoff(altitude)
        
        if wait:
            while self.vehicle.location.global_relative_frame.alt < altitude * 0.95:
                logger.info(f"상승 중... {self.vehicle.location.global_relative_frame.alt:.1f}m")
                time.sleep(1)
            
            logger.info("이륙 완료")
            self.hover()
        
        return True
    
    def land(self, wait: bool = True) -> bool:
        """
        착륙
        
        Args:
            wait: 착륙 완료 대기 여부
        
        Returns:
            성공 여부
        """
        if not self.vehicle:
            return False
        
        logger.info("착륙 시작")
        self.vehicle.mode = VehicleMode("LAND")
        
        if wait:
            while self.vehicle.armed:
                alt = self.vehicle.location.global_relative_frame.alt
                logger.info(f"하강 중... {alt:.1f}m")
                time.sleep(1)
            
            logger.info("착륙 완료")
        
        return True
    
    def hover(self):
        """호버링"""
        self.set_command("level", "hover", 0, THROTTLE_HOVER)
    
    def emergency_stop(self):
        """긴급 정지"""
        logger.warning("긴급 정지!")
        self.gps_navigation_active = False
        self.set_command("level", "hover", 0, 0)
        self.vehicle.mode = VehicleMode("BRAKE")
    
    # ========================= 편의 메서드 =========================
    
    def forward(self, speed: float = 50):
        self.set_command("level", "forward", 0, speed)
    
    def backward(self, speed: float = 50):
        self.set_command("level", "backward", 0, speed)
    
    def left(self, speed: float = 50):
        self.set_command("level", "left", 0, speed)
    
    def right(self, speed: float = 50):
        self.set_command("level", "right", 0, speed)
    
    def up(self, speed: float = 50):
        self.set_command("up", "hover", 0, speed)
    
    def down(self, speed: float = 50):
        self.set_command("down", "hover", 0, speed)
    
    def rotate(self, degree: float):
        self.set_command("level", "hover", degree, 0)
    
    def diagonal(self, direction: str, speed: float = 50):
        self.set_command("level", direction, 0, speed)
    
    # ========================= 상태 정보 =========================
    
    def get_status(self) -> DroneStatus:
        """현재 상태 반환"""
        if not self.vehicle:
            return DroneStatus(
                connected=False, armed=False, mode="N/A",
                latitude=0, longitude=0, altitude=0, heading=0,
                battery_voltage=0, battery_level=0, gps_satellites=0,
                current_command=self.current_cmd, is_flying=False,
                home_location=self.home_gps if self.home_gps else (0, 0, 0)
            )
        
        return DroneStatus(
            connected=True,
            armed=self.vehicle.armed,
            mode=self.vehicle.mode.name,
            latitude=self.vehicle.location.global_frame.lat,
            longitude=self.vehicle.location.global_frame.lon,
            altitude=self.vehicle.location.global_relative_frame.alt,
            heading=self.vehicle.heading,
            battery_voltage=self.vehicle.battery.voltage,
            battery_level=self.vehicle.battery.level if self.vehicle.battery.level else 0,
            gps_satellites=self.vehicle.gps_0.satellites_visible,
            current_command=self.current_cmd,
            is_flying=self.vehicle.location.global_relative_frame.alt > 0.5,
            home_location=self.home_gps if self.home_gps else (0, 0, 0)
        )
    
    def get_flight_stats(self) -> FlightStats:
        """비행 통계 반환"""
        return self.flight_stats
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.vehicle is not None
    
    def is_armed(self) -> bool:
        """시동 상태 확인"""
        return self.vehicle.armed if self.vehicle else False
    
    def is_flying(self) -> bool:
        """비행 중 확인"""
        if not self.vehicle:
            return False
        return self.vehicle.location.global_relative_frame.alt > 0.5
    
    # ========================= 내부 루프 =========================
    
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
                
                time.sleep(0.2)
                
            except Exception as e:
                logger.error(f"GPS 업데이트 오류: {e}")
                time.sleep(1)
    
    def _navigation_loop(self):
        """GPS 네비게이션 루프"""
        while self.is_running:
            try:
                if self.gps_navigation_active and self.target_gps and self.current_gps:
                    self._navigate_to_target()
                
                time.sleep(0.5)
                
            except Exception as e:
                logger.error(f"네비게이션 오류: {e}")
                time.sleep(1)
    
    def _monitor_loop(self):
        """시스템 모니터링 루프"""
        while self.is_running:
            try:
                if self.vehicle:
                    # 최대 고도 기록
                    current_alt = self.vehicle.location.global_relative_frame.alt
                    if current_alt > self.flight_stats.max_altitude:
                        self.flight_stats.max_altitude = current_alt
                    
                    # 고도 제한 확인
                    if current_alt > MAX_ALTITUDE:
                        logger.warning(f"최대 고도 초과: {current_alt:.1f}m")
                        self.set_command("down", "hover", 0, THROTTLE_LAND)
                
                time.sleep(1)
                
            except Exception as e:
                logger.error(f"모니터링 오류: {e}")
                time.sleep(1)
    
    def _navigate_to_target(self):
        """목표 GPS로 네비게이션"""
        cur_lat, cur_lon, cur_alt = self.current_gps
        tgt_lat, tgt_lon, tgt_alt = self.target_gps
        
        # 거리 계산
        distance = self._calculate_distance(cur_lat, cur_lon, tgt_lat, tgt_lon)
        self.flight_stats.total_distance += distance * 0.5  # 대략적인 거리 누적
        
        # 도착 확인
        if distance < GPS_REACH_THRESHOLD:
            logger.info(f"목표 도달! (거리: {distance:.1f}m)")
            self.gps_navigation_active = False
            self.flight_stats.waypoints_visited += 1
            self.hover()
            
            if self.on_gps_reached:
                self.on_gps_reached(tgt_lat, tgt_lon)
            return
        
        # 방향 계산
        bearing = self._calculate_bearing(cur_lat, cur_lon, tgt_lat, tgt_lon)
        current_heading = self.vehicle.heading
        
        # 회전 각도
        rotation_needed = bearing - current_heading
        if rotation_needed > 180:
            rotation_needed -= 360
        elif rotation_needed < -180:
            rotation_needed += 360
        
        # 방향 결정
        L2 = "forward"
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
        
        # 속도
        L4 = min(MAX_SPEED_GPS, max(30, distance * 5))
        
        # 회전
        L3 = abs(rotation_needed) if abs(rotation_needed) > 10 else 0
        
        # 명령 전송
        self.set_command(L1, L2, L3, L4)
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """GPS 거리 계산"""
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def _calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """GPS 방위각 계산"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)
        
        x = math.sin(dlambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
        
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360
