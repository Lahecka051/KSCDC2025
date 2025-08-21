"""
integrated_drone_system.py
FC와 Jetson 통합 드론 제어 시스템
4가지 데이터 제어 + GPS 이동
"""

import serial
import json
import threading
import queue
import time
import math
import logging
from typing import List, Tuple, Optional, Dict, Any, Callable
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import Enum

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

# ========================= 데이터 클래스 =========================

@dataclass
class GPSData:
    """GPS 데이터"""
    latitude: float
    longitude: float
    altitude: float
    heading: float
    speed: float
    satellites: int
    timestamp: float

@dataclass
class DroneCommand:
    """드론 제어 명령 (4가지 데이터)"""
    vertical: str          # up/level/down
    horizontal: str        # 8방향 + hover
    rotation: float        # 0-359도
    speed: float          # 0-100% 또는 m/s
    speed_type: str = "percent"  # percent/ms

class ControlMode(Enum):
    """제어 모드"""
    MANUAL = "manual"
    GPS = "gps"
    AUTO = "auto"

# ========================= 메인 드론 클래스 =========================

class IntegratedDroneSystem:
    """통합 드론 제어 시스템"""
    
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
        
        # 제어 모드
        self.mode = ControlMode.MANUAL
        
        # 현재 명령 (4가지 데이터)
        self._current_cmd = ["level", "hover", 0, 0]
        self.cmd_lock = threading.Lock()
        
        # GPS 데이터
        self.current_gps = None
        self.gps_lock = threading.Lock()
        self.home_position = None
        self.target_position = None
        
        # 통신 상태
        self.is_connected = False
        self.is_running = False
        
        # 스레드
        self.rx_thread = None
        self.control_thread = None
        
        # 큐
        self.command_queue = queue.Queue(maxsize=100)
        
        # 콜백
        self.gps_callback = None
        
        # 파라미터
        self.POSITION_THRESHOLD = 3.0  # GPS 도달 판정 거리 (m)
        self.MAX_SPEED = 10.0  # 최대 속도 (m/s)
        self.DEFAULT_SPEED = 50.0  # 기본 속도 (%)
        
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
            
            self.is_connected = True
            self.is_running = True
            
            # 스레드 시작
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.rx_thread.start()
            self.control_thread.start()
            
            self.logger.info(f"드론 시스템 연결: {self.port}@{self.baudrate}")
            return True
            
        except Exception as e:
            self.logger.error(f"연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.is_running = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=1)
        if self.control_thread:
            self.control_thread.join(timeout=1)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.is_connected = False
        self.logger.info("드론 시스템 연결 해제")
    
    # ========================= 명령 설정 (4가지 데이터) =========================
    
    def set_command(self, vertical: str, horizontal: str, rotation: float, speed: float):
        """
        드론 제어 명령 설정 (4가지 데이터)
        
        Args:
            vertical: up/level/down
            horizontal: forward/backward/left/right/forward_left/forward_right/backward_left/backward_right/hover
            rotation: 0-359 (시계방향)
            speed: 0-100 (%)
        """
        # 입력 검증
        valid_vertical = ["up", "level", "down"]
        valid_horizontal = [
            "forward", "backward", "left", "right",
            "forward_left", "forward_right", 
            "backward_left", "backward_right", "hover"
        ]
        
        if vertical not in valid_vertical:
            self.logger.error(f"잘못된 vertical: {vertical}")
            return False
        
        if horizontal not in valid_horizontal:
            self.logger.error(f"잘못된 horizontal: {horizontal}")
            return False
        
        # 회전 각도 정규화 (0-359)
        rotation = rotation % 360
        
        # 속도 제한 (0-100)
        speed = max(0, min(100, speed))
        
        # 명령 설정
        with self.cmd_lock:
            self._current_cmd = [vertical, horizontal, rotation, speed]
        
        self.logger.debug(f"명령 설정: [{vertical}, {horizontal}, {rotation:.1f}°, {speed:.1f}%]")
        
        # FC로 전송
        self._send_control_command(vertical, horizontal, rotation, speed)
        
        return True
    
    def get_command(self) -> List:
        """현재 명령 반환"""
        with self.cmd_lock:
            return self._current_cmd.copy()
    
    # ========================= GPS 이동 =========================
    
    def goto_gps(self, latitude: float, longitude: float, altitude: float = None):
        """
        GPS 좌표로 이동
        
        Args:
            latitude: 목표 위도
            longitude: 목표 경도
            altitude: 목표 고도 (None이면 현재 고도 유지)
        """
        self.mode = ControlMode.GPS
        self.target_position = (latitude, longitude, altitude)
        
        self.logger.info(f"GPS 이동 시작: ({latitude:.6f}, {longitude:.6f})")
        
        # GPS 이동 제어는 _control_loop에서 처리
    
    def set_home(self, latitude: float = None, longitude: float = None):
        """
        홈 위치 설정
        
        Args:
            latitude: 위도 (None이면 현재 위치)
            longitude: 경도 (None이면 현재 위치)
        """
        if latitude is None and self.current_gps:
            self.home_position = (self.current_gps.latitude, self.current_gps.longitude)
            self.logger.info(f"홈 설정 (현재): ({self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f})")
        else:
            self.home_position = (latitude, longitude)
            self.logger.info(f"홈 설정: ({latitude:.6f}, {longitude:.6f})")
    
    def return_home(self):
        """홈으로 복귀"""
        if self.home_position:
            self.goto_gps(self.home_position[0], self.home_position[1])
            self.logger.info("홈 복귀 시작")
        else:
            self.logger.warning("홈 위치가 설정되지 않음")
    
    # ========================= 편의 메서드 =========================
    
    def move(self, vertical: str, horizontal: str, rotation: float = 0, speed: float = None):
        """간편 이동 명령"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command(vertical, horizontal, rotation, speed)
    
    def hover(self):
        """호버링"""
        return self.set_command("level", "hover", 0, 0)
    
    def forward(self, speed: float = None):
        """전진"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("level", "forward", 0, speed)
    
    def backward(self, speed: float = None):
        """후진"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("level", "backward", 0, speed)
    
    def left(self, speed: float = None):
        """좌측"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("level", "left", 0, speed)
    
    def right(self, speed: float = None):
        """우측"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("level", "right", 0, speed)
    
    def up(self, speed: float = None):
        """상승"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("up", "hover", 0, speed)
    
    def down(self, speed: float = None):
        """하강"""
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("down", "hover", 0, speed)
    
    def rotate(self, angle: float):
        """회전 (시계방향)"""
        return self.set_command("level", "hover", angle, 0)
    
    def diagonal(self, direction: str, speed: float = None):
        """
        대각선 이동
        
        Args:
            direction: forward_left/forward_right/backward_left/backward_right
            speed: 속도
        """
        if speed is None:
            speed = self.DEFAULT_SPEED
        return self.set_command("level", direction, 0, speed)
    
    def emergency_stop(self):
        """긴급 정지"""
        self.logger.warning("긴급 정지!")
        self.mode = ControlMode.MANUAL
        return self.set_command("level", "hover", 0, 0)
    
    # ========================= 내부 메서드 =========================
    
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
    
    def _control_loop(self):
        """제어 루프 (GPS 이동 처리)"""
        while self.is_running:
            try:
                if self.mode == ControlMode.GPS and self.target_position:
                    self._process_gps_navigation()
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.logger.error(f"제어 루프 오류: {e}")
                time.sleep(0.5)
    
    def _process_message(self, message: str):
        """메시지 처리"""
        try:
            data = json.loads(message)
            
            if data.get('type') == 'GPS':
                self._handle_gps_data(data.get('data', {}))
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
    
    def _handle_gps_data(self, data: Dict):
        """GPS 데이터 처리"""
        try:
            gps = GPSData(
                latitude=data.get('lat', 0.0),
                longitude=data.get('lon', 0.0),
                altitude=data.get('alt', 0.0),
                heading=data.get('heading', 0.0),
                speed=data.get('speed', 0.0),
                satellites=data.get('sats', 0),
                timestamp=time.time()
            )
            
            with self.gps_lock:
                self.current_gps = gps
            
            # 콜백 호출
            if self.gps_callback:
                self.gps_callback(gps)
                
        except Exception as e:
            self.logger.error(f"GPS 처리 오류: {e}")
    
    def _process_gps_navigation(self):
        """GPS 네비게이션 처리"""
        if not self.current_gps or not self.target_position:
            return
        
        target_lat, target_lon, target_alt = self.target_position
        
        # 현재 위치와 목표 거리 계산
        distance = self._calculate_distance(
            self.current_gps.latitude, self.current_gps.longitude,
            target_lat, target_lon
        )
        
        # 도착 확인
        if distance < self.POSITION_THRESHOLD:
            self.logger.info(f"목표 도달! 거리: {distance:.1f}m")
            self.mode = ControlMode.MANUAL
            self.hover()
            return
        
        # 목표 방향 계산
        bearing = self._calculate_bearing(
            self.current_gps.latitude, self.current_gps.longitude,
            target_lat, target_lon
        )
        
        # 회전 각도 계산
        rotation = bearing - self.current_gps.heading
        if rotation > 180:
            rotation -= 360
        elif rotation < -180:
            rotation += 360
        
        # 이동 방향 결정
        if abs(rotation) < 30:
            # 전진
            horizontal = "forward"
        elif abs(rotation) < 60:
            # 대각선
            if rotation > 0:
                horizontal = "forward_right"
            else:
                horizontal = "forward_left"
        elif abs(rotation) < 120:
            # 좌우
            if rotation > 0:
                horizontal = "right"
            else:
                horizontal = "left"
        else:
            # 후진
            horizontal = "backward"
        
        # 고도 조절
        vertical = "level"
        if target_alt and self.current_gps.altitude:
            alt_diff = target_alt - self.current_gps.altitude
            if alt_diff > 1.0:
                vertical = "up"
            elif alt_diff < -1.0:
                vertical = "down"
        
        # 속도 결정 (거리에 비례)
        speed = min(80, max(30, distance * 5))
        
        # 명령 전송
        self.set_command(vertical, horizontal, abs(rotation), speed)
        
        self.logger.debug(f"GPS Nav: 거리={distance:.1f}m, 방향={bearing:.0f}°, 회전={rotation:.0f}°")
    
    def _send_control_command(self, vertical: str, horizontal: str, rotation: float, speed: float):
        """FC로 제어 명령 전송"""
        try:
            command = {
                'type': 'CONTROL',
                'data': {
                    'vertical': vertical,
                    'horizontal': horizontal,
                    'rotation': rotation,
                    'speed': speed,
                    'timestamp': datetime.now().isoformat()
                }
            }
            
            if self.serial and self.serial.is_open:
                json_str = json.dumps(command) + '\n'
                self.serial.write(json_str.encode('utf-8'))
                
        except Exception as e:
            self.logger.error(f"명령 전송 오류: {e}")
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """GPS 좌표 간 거리 계산 (미터)"""
        R = 6371000  # 지구 반경
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def _calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """GPS 좌표 간 방위각 계산 (0-359도)"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.degrees(math.atan2(x, y))
        
        return (bearing + 360) % 360
    
    # ========================= GPS 콜백 =========================
    
    def set_gps_callback(self, callback: Callable):
        """GPS 업데이트 콜백 설정"""
        self.gps_callback = callback
    
    def get_gps(self) -> Optional[GPSData]:
        """현재 GPS 데이터 반환"""
        with self.gps_lock:
            return self.current_gps


# ========================= 사용 예제 =========================

def example_usage():
    """사용 예제"""
    
    # 드론 시스템 생성
    drone = IntegratedDroneSystem("/dev/ttyTHS1", 115200)
    
    # 연결
    if not drone.connect():
        print("드론 연결 실패!")
        return
    
    try:
        # 1. 4가지 데이터로 제어
        print("\n=== 4가지 데이터 제어 ===")
        
        # 전진
        drone.set_command("level", "forward", 0, 50)
        time.sleep(3)
        
        # 우상향 대각선 + 회전
        drone.set_command("up", "forward_right", 45, 60)
        time.sleep(3)
        
        # 좌측 이동 + 시계방향 90도 회전
        drone.set_command("level", "left", 90, 40)
        time.sleep(3)
        
        # 하강하며 후진
        drone.set_command("down", "backward", 0, 30)
        time.sleep(3)
        
        # 호버링
        drone.hover()
        time.sleep(2)
        
        # 2. 간편 메서드 사용
        print("\n=== 간편 제어 ===")
        
        drone.forward(60)
        time.sleep(2)
        
        drone.diagonal("forward_left", 50)
        time.sleep(2)
        
        drone.rotate(180)
        time.sleep(2)
        
        drone.up(40)
        time.sleep(2)
        
        # 3. GPS 이동
        print("\n=== GPS 이동 ===")
        
        # 현재 위치를 홈으로 설정
        drone.set_home()
        
        # GPS 좌표로 이동
        drone.goto_gps(35.123456, 129.123456)
        
        # 이동 완료 대기
        while drone.mode == ControlMode.GPS:
            time.sleep(1)
            current = drone.get_gps()
            if current:
                print(f"현재: ({current.latitude:.6f}, {current.longitude:.6f})")
        
        # 홈 복귀
        drone.return_home()
        
        # 4. 현재 명령 확인
        current_cmd = drone.get_command()
        print(f"\n현재 명령: {current_cmd}")
        
    except KeyboardInterrupt:
        print("\n긴급 정지!")
        drone.emergency_stop()
    
    finally:
        drone.disconnect()


if __name__ == "__main__":
    print("통합 드론 제어 시스템")
    print("=" * 50)
    example_usage()
