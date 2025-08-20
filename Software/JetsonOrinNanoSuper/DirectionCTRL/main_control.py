"""
main_control.py - 실전 최종 버전
H743v2 FC 드론의 메인 컨트롤 프로그램
통합 제어, 객체인식 연동, GPS 자율비행, 안전 관리
"""

import asyncio
import threading
import queue
import time
import logging
import signal
import sys
import json
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Any, Tuple, Callable
from datetime import datetime
import numpy as np

# 모듈 임포트
from drone_communication import UARTCommunication, MessageType
from drone_control import DroneConnection, DroneController, ControlMode, FlightStatus, SafetyLimits

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(name)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(f'drone_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
    ]
)

class FlightMode(Enum):
    """비행 모드"""
    IDLE = "idle"                     # 대기
    MANUAL = "manual"                 # 수동 제어
    OBJECT_TRACKING = "object_tracking"  # 객체 추적
    GPS_WAYPOINT = "gps_waypoint"    # GPS 웨이포인트
    GPS_FOLLOW = "gps_follow"        # GPS 경로 추적
    PATROL = "patrol"                 # 순찰
    LANDING = "landing"               # 착륙
    EMERGENCY = "emergency"           # 긴급
    RETURN_HOME = "return_home"       # 복귀

@dataclass
class ObjectTarget:
    """추적 대상 객체"""
    class_name: str
    confidence: float
    center_x: int
    center_y: int
    width: int
    height: int
    distance: Optional[float] = None
    timestamp: float = 0.0

@dataclass
class Waypoint:
    """GPS 웨이포인트"""
    latitude: float
    longitude: float
    altitude: float
    speed: float = 5.0
    hold_time: float = 0.0  # 호버링 시간
    action: Optional[str] = None  # 도착 시 수행할 액션

@dataclass
class MissionStatus:
    """미션 상태"""
    mode: str = "idle"
    is_armed: bool = False
    is_flying: bool = False
    current_waypoint: int = 0
    total_waypoints: int = 0
    battery_percent: float = 0.0
    altitude: float = 0.0
    gps_satellites: int = 0
    flight_time: float = 0.0
    distance_traveled: float = 0.0
    errors: int = 0

class PIDController:
    """PID 제어기"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=1.0, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral_limit = integral_limit
        
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
    
    def update(self, error: float) -> float:
        """PID 제어 업데이트"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.01
        
        # P 항
        p_term = self.kp * error
        
        # I 항 (적분 제한)
        self.error_sum += error * dt
        self.error_sum = max(-self.integral_limit, min(self.integral_limit, self.error_sum))
        i_term = self.ki * self.error_sum
        
        # D 항
        d_term = self.kd * (error - self.last_error) / dt
        
        # 전체 출력
        output = p_term + i_term + d_term
        
        # 출력 제한
        output = max(-self.max_output, min(self.max_output, output))
        
        # 상태 업데이트
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID 상태 초기화"""
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

class Drone:
    """메인 드론 컨트롤러"""
    
    def __init__(self, config_file: Optional[str] = None):
        """
        초기화
        
        Args:
            config_file: 설정 파일 경로
        """
        self.logger = logging.getLogger(__name__)
        
        # 설정 로드
        self.config = self._load_config(config_file)
        
        # 드론 연결
        self.connection = DroneConnection(self.config['fc_connection'])
        self.controller = None
        
        # UART 통신
        self.uart = UARTCommunication(
            self.config['uart_port'],
            self.config['uart_baudrate']
        )
        
        # 현재 명령
        self._cmd = ["level", "hover", 0, 0]
        self._cmd_lock = threading.Lock()
        
        # 비행 모드
        self.flight_mode = FlightMode.IDLE
        
        # 미션 상태
        self.mission_status = MissionStatus()
        
        # 웨이포인트
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # 객체 추적
        self.tracking_target = None
        self.camera_config = self.config.get('camera', {
            'width': 640,
            'height': 480,
            'fov_h': 62.0,
            'fov_v': 48.0
        })
        
        # PID 제어기 (객체 추적용)
        self.pid_x = PIDController(kp=0.5, ki=0.01, kd=0.1, max_output=0.8)
        self.pid_y = PIDController(kp=0.3, ki=0.01, kd=0.05, max_output=0.5)
        self.pid_z = PIDController(kp=0.4, ki=0.01, kd=0.08, max_output=0.6)
        
        # 제어 큐
        self.command_queue = queue.PriorityQueue(maxsize=100)
        
        # 상태 플래그
        self.is_running = False
        self.is_initialized = False
        
        # 태스크
        self.control_task = None
        self.gps_task = None
        self.uart_task = None
        self.monitor_task = None
        
        # 통계
        self.stats = {
            'flight_time': 0.0,
            'distance_traveled': 0.0,
            'commands_executed': 0,
            'waypoints_reached': 0,
            'objects_tracked': 0,
            'errors': 0
        }
        
        # 비행 시작 시간
        self.flight_start_time = None
        
        # 이전 위치 (거리 계산용)
        self.last_position = None
        
        # 블랙박스 (비행 기록)
        self.flight_log = []
        self.max_log_size = 10000
        
    def _load_config(self, config_file: Optional[str]) -> Dict:
        """설정 로드"""
        default_config = {
            'fc_connection': 'serial:///dev/ttyTHS0:115200',
            'uart_port': '/dev/ttyTHS1',
            'uart_baudrate': 115200,
            'max_altitude': 50.0,
            'max_distance': 200.0,
            'min_battery': 20.0,
            'max_speed': 10.0,
            'geofence_radius': 300.0,
            'home_lat': 0.0,
            'home_lon': 0.0
        }
        
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                    self.logger.info(f"설정 파일 로드: {config_file}")
            except Exception as e:
                self.logger.warning(f"설정 파일 로드 실패: {e}")
        
        return default_config
    
    @property
    def cmd(self):
        """현재 명령 반환"""
        with self._cmd_lock:
            return self._cmd.copy()
    
    @cmd.setter
    def cmd(self, value: List):
        """
        명령 설정
        
        Args:
            value: 제어 명령
                - 일반: [vertical, horizontal, rotation, motor_percent]
                - GPS: ["gps", latitude, longitude, altitude(선택)]
        """
        # 검증
        if not self._validate_command(value):
            return
        
        with self._cmd_lock:
            self._cmd = value
            self.logger.debug(f"명령 설정: {value}")
        
        # 블랙박스 기록
        self._log_flight_data('command', {'cmd': value})
    
    def _validate_command(self, cmd: List) -> bool:
        """명령 검증"""
        if len(cmd) < 3 or len(cmd) > 4:
            self.logger.error(f"잘못된 명령 길이: {len(cmd)}")
            return False
        
        # GPS 명령
        if cmd[0] == "gps":
            try:
                lat = float(cmd[1])
                lon = float(cmd[2])
                if len(cmd) == 4:
                    alt = float(cmd[3])
                    if alt < 0 or alt > self.config['max_altitude']:
                        self.logger.warning(f"고도 제한: {alt}m")
                        return False
                return True
            except (ValueError, TypeError):
                self.logger.error("GPS 좌표 형식 오류")
                return False
        
        # 일반 명령
        if len(cmd) != 4:
            self.logger.error(f"일반 명령은 4개 값 필요")
            return False
        
        valid_vertical = ["up", "level", "down"]
        valid_horizontal = ["forward", "backward", "left", "right",
                          "forward_left", "forward_right",
                          "backward_left", "backward_right", "hover"]
        
        if cmd[0] not in valid_vertical:
            self.logger.error(f"잘못된 수직 방향: {cmd[0]}")
            return False
        
        if cmd[1] not in valid_horizontal:
            self.logger.error(f"잘못된 수평 방향: {cmd[1]}")
            return False
        
        try:
            rotation = float(cmd[2])
            if rotation < -180 or rotation > 360:
                self.logger.warning(f"회전 각도 제한: {rotation}")
                return False
            
            speed = float(cmd[3])
            if speed < 0 or speed > 100:
                self.logger.warning(f"속도 제한: {speed}%")
                return False
            
        except (ValueError, TypeError):
            self.logger.error("숫자 형식 오류")
            return False
        
        return True
    
    async def initialize(self) -> bool:
        """시스템 초기화"""
        self.logger.info("="*60)
        self.logger.info("드론 시스템 초기화")
        self.logger.info("="*60)
        
        try:
            # FC 연결
            self.logger.info("1. FC 연결...")
            if not await self.connection.connect():
                raise Exception("FC 연결 실패")
            
            # 컨트롤러 생성
            self.controller = DroneController(self.connection)
            
            # 안전 제한 설정
            self.controller.set_safety_limits(
                max_altitude=self.config['max_altitude'],
                max_distance=self.config['max_distance'],
                min_battery=self.config['min_battery'],
                geofence_radius=self.config['geofence_radius']
            )
            
            # Home 위치 설정
            if self.config['home_lat'] and self.config['home_lon']:
                self.controller.set_home_position(
                    self.config['home_lat'],
                    self.config['home_lon']
                )
            
            # 긴급 콜백 설정
            self.controller.set_emergency_callback(self._on_emergency)
            
            # UART 시작
            self.logger.info("2. UART 통신...")
            if not self.uart.start():
                raise Exception("UART 시작 실패")
            
            # 콜백 설정
            self.uart.set_callback(MessageType.CONTROL_CMD, self._on_control_command)
            self.uart.set_callback(MessageType.GPS_CMD, self._on_gps_command)
            self.uart.set_callback(MessageType.CONFIG, self._on_config_update)
            
            self.is_running = True
            self.is_initialized = True
            
            # 태스크 시작
            self.control_task = asyncio.create_task(self._control_loop())
            self.gps_task = asyncio.create_task(self._gps_loop())
            self.uart_task = asyncio.create_task(self._uart_loop())
            self.monitor_task = asyncio.create_task(self._monitor_loop())
            
            self.logger.info("3. 초기화 완료!")
            self.logger.info("="*60)
            
            # 초기 상태 전송
            await self._send_status()
            
            return True
            
        except Exception as e:
            self.logger.error(f"초기화 실패: {e}")
            self.is_initialized = False
            return False
    
    async def _control_loop(self):
        """제어 루프"""
        last_cmd_time = time.time()
        
        while self.is_running:
            try:
                # 비행 중일 때만 명령 실행
                if self.mission_status.is_flying:
                    # 우선순위 큐 확인
                    try:
                        priority, timestamp, cmd = self.command_queue.get_nowait()
                        self._cmd = cmd
                    except queue.Empty:
                        pass
                    
                    # 모드별 처리
                    if self.flight_mode == FlightMode.OBJECT_TRACKING:
                        await self._process_tracking()
                    elif self.flight_mode == FlightMode.GPS_WAYPOINT:
                        await self._process_waypoint()
                    elif self.flight_mode == FlightMode.PATROL:
                        await self._process_patrol()
                    elif self.flight_mode == FlightMode.MANUAL:
                        # 현재 명령 실행
                        current_cmd = self.cmd
                        success = await self.controller.execute_cmd(current_cmd)
                        
                        if success:
                            self.stats['commands_executed'] += 1
                        else:
                            self.stats['errors'] += 1
                    
                    # 명령 타임아웃 체크
                    if time.time() - last_cmd_time > 1.0:
                        if self.flight_mode == FlightMode.MANUAL:
                            # 타임아웃 시 호버링
                            self.cmd = ["level", "hover", 0, 0]
                    
                    last_cmd_time = time.time()
                
                await asyncio.sleep(0.05)  # 20Hz
                
            except Exception as e:
                self.logger.error(f"제어 루프 오류: {e}")
                self.stats['errors'] += 1
                await asyncio.sleep(0.1)
    
    async def _gps_loop(self):
        """GPS 데이터 전송"""
        while self.is_running:
            try:
                # 텔레메트리 수집
                telemetry = await self.connection.get_telemetry()
                
                if telemetry:
                    # UART로 전송
                    self.uart.send_gps_data(telemetry)
                    
                    # 거리 계산
                    if 'position' in telemetry:
                        current_pos = telemetry['position']
                        if self.last_position:
                            distance = self._calculate_distance(
                                self.last_position['lat'], self.last_position['lon'],
                                current_pos['lat'], current_pos['lon']
                            )
                            self.stats['distance_traveled'] += distance
                        self.last_position = current_pos
                    
                    # 미션 상태 업데이트
                    self._update_mission_status(telemetry)
                
                await asyncio.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.logger.error(f"GPS 루프 오류: {e}")
                await asyncio.sleep(1)
    
    async def _uart_loop(self):
        """UART 명령 처리"""
        while self.is_running:
            try:
                # 제어 명령 처리
                cmd = self.uart.get_control_command(timeout=0.05)
                if cmd:
                    command_data = cmd.get('command', {})
                    self.add_command(
                        vertical=command_data.get('vertical', 'level'),
                        horizontal=command_data.get('horizontal', 'hover'),
                        rotation=command_data.get('rotation', 0),
                        speed=command_data.get('speed', 0),
                        priority=5,
                        source='uart'
                    )
                
                # GPS 명령 처리
                gps_cmd = self.uart.get_gps_command(timeout=0.05)
                if gps_cmd:
                    gps_data = gps_cmd.get('command', {})
                    if 'altitude' in gps_data:
                        self.cmd = ["gps", gps_data['latitude'],
                                   gps_data['longitude'], gps_data['altitude']]
                    else:
                        self.cmd = ["gps", gps_data['latitude'],
                                   gps_data['longitude']]
                
                await asyncio.sleep(0.01)
                
            except Exception as e:
                self.logger.error(f"UART 루프 오류: {e}")
                await asyncio.sleep(0.1)
    
    async def _monitor_loop(self):
        """모니터링 루프"""
        while self.is_running:
            try:
                # 상태 전송
                await self._send_status()
                
                # 비행 시간 업데이트
                if self.flight_start_time:
                    self.stats['flight_time'] = time.time() - self.flight_start_time
                
                # 안전 체크
                await self._safety_check()
                
                # 통신 상태 체크
                if not self.uart.is_healthy():
                    self.logger.warning("UART 통신 상태 불량")
                
                # 로그 크기 관리
                if len(self.flight_log) > self.max_log_size:
                    self.flight_log = self.flight_log[-self.max_log_size//2:]
                
                await asyncio.sleep(1.0)  # 1Hz
                
            except Exception as e:
                self.logger.error(f"모니터링 오류: {e}")
                await asyncio.sleep(1)
    
    async def _process_tracking(self):
        """객체 추적 처리"""
        if not self.tracking_target:
            return
        
        try:
            # 화면 중심과의 오차
            center_x = self.camera_config['width'] // 2
            center_y = self.camera_config['height'] // 2
            
            error_x = self.tracking_target.center_x - center_x
            error_y = self.tracking_target.center_y - center_y
            
            # 정규화
            norm_x = error_x / center_x
            norm_y = error_y / center_y
            
            # PID 제어
            control_x = self.pid_x.update(norm_x)
            control_y = self.pid_y.update(norm_y)
            
            # 명령 생성
            horizontal = "hover"
            vertical = "level"
            rotation = 0.0
            speed = 30.0
            
            # 수평 이동
            if abs(control_x) > 0.1:
                if control_x > 0:
                    horizontal = "right"
                else:
                    horizontal = "left"
                speed = min(abs(control_x) * 60, 70)
            
            # 수직 이동
            if abs(control_y) > 0.1:
                if control_y > 0:
                    vertical = "down"
                else:
                    vertical = "up"
            
            # 회전
            if abs(norm_x) > 0.5:
                rotation = control_x * 45
            
            # 거리 기반 전후
            if self.tracking_target.distance:
                if self.tracking_target.distance > 3.0:
                    if horizontal == "hover":
                        horizontal = "forward"
                elif self.tracking_target.distance < 1.5:
                    if horizontal == "hover":
                        horizontal = "backward"
            
            # 명령 설정
            self.cmd = [vertical, horizontal, rotation, speed]
            
            self.stats['objects_tracked'] += 1
            
        except Exception as e:
            self.logger.error(f"추적 처리 오류: {e}")
    
    async def _process_waypoint(self):
        """웨이포인트 처리"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.logger.info("모든 웨이포인트 완료")
            self.flight_mode = FlightMode.MANUAL
            return
        
        try:
            waypoint = self.waypoints[self.current_waypoint_idx]
            
            # GPS 이동
            self.cmd = ["gps", waypoint.latitude, waypoint.longitude, waypoint.altitude]
            
            # 도착 확인
            telemetry = await self.connection.get_telemetry()
            current_pos = telemetry.get('position', {})
            
            distance = self._calculate_distance(
                current_pos.get('lat', 0), current_pos.get('lon', 0),
                waypoint.latitude, waypoint.longitude
            )
            
            if distance < 3.0:  # 3m 이내 도착
                self.logger.info(f"웨이포인트 {self.current_waypoint_idx + 1} 도착")
                
                # 홀드 시간
                if waypoint.hold_time > 0:
                    self.cmd = ["level", "hover", 0, 0]
                    await asyncio.sleep(waypoint.hold_time)
                
                # 액션 실행
                if waypoint.action:
                    await self._execute_waypoint_action(waypoint.action)
                
                self.current_waypoint_idx += 1
                self.stats['waypoints_reached'] += 1
                
        except Exception as e:
            self.logger.error(f"웨이포인트 처리 오류: {e}")
    
    async def _process_patrol(self):
        """순찰 모드 처리"""
        # 웨이포인트를 반복
        await self._process_waypoint()
        
        # 마지막 웨이포인트 도달 시 처음으로
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_waypoint_idx = 0
    
    async def _execute_waypoint_action(self, action: str):
        """웨이포인트 액션 실행"""
        self.logger.info(f"액션 실행: {action}")
        
        if action == "photo":
            # 사진 촬영 (구현 필요)
            pass
        elif action == "rotate_360":
            # 360도 회전
            for angle in range(0, 360, 45):
                self.cmd = ["level", "hover", angle, 0]
                await asyncio.sleep(1)
    
    def add_command(self, vertical="level", horizontal="hover",
                   rotation=0, speed=0, priority=5, source="manual"):
        """명령 추가"""
        cmd = [vertical, horizontal, rotation, speed]
        
        if self._validate_command(cmd):
            self.command_queue.put((priority, time.time(), cmd))
            self.logger.debug(f"명령 추가: {source} - {cmd} (우선순위: {priority})")
    
    def add_waypoint(self, latitude: float, longitude: float,
                    altitude: float, speed: float = 5.0,
                    hold_time: float = 0.0, action: Optional[str] = None):
        """웨이포인트 추가"""
        waypoint = Waypoint(latitude, longitude, altitude, speed, hold_time, action)
        self.waypoints.append(waypoint)
        self.logger.info(f"웨이포인트 추가: {len(self.waypoints)}번째")
    
    def set_tracking_target(self, target: ObjectTarget):
        """추적 대상 설정"""
        self.tracking_target = target
        
        if self.flight_mode != FlightMode.OBJECT_TRACKING:
            self.logger.info(f"객체 추적 시작: {target.class_name}")
            self.flight_mode = FlightMode.OBJECT_TRACKING
            
            # PID 리셋
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
    
    def set_flight_mode(self, mode: FlightMode):
        """비행 모드 설정"""
        self.flight_mode = mode
        self.logger.info(f"비행 모드 변경: {mode.value}")
        
        # 모드별 초기화
        if mode == FlightMode.GPS_WAYPOINT:
            self.current_waypoint_idx = 0
        elif mode == FlightMode.MANUAL:
            self.tracking_target = None
    
    async def arm_and_takeoff(self, altitude: float = 2.0) -> bool:
        """시동 및 이륙"""
        try:
            self.logger.info("시동 및 이륙 시작...")
            
            # 시동
            if not await self.connection.arm():
                return False
            
            self.mission_status.is_armed = True
            await asyncio.sleep(2)
            
            # 이륙
            if not await self.connection.takeoff(altitude):
                await self.connection.disarm()
                return False
            
            await asyncio.sleep(5)
            
            # Offboard 모드
            await self.controller.start_offboard()
            
            # 상태 업데이트
            self.mission_status.is_flying = True
            self.flight_mode = FlightMode.MANUAL
            self.flight_start_time = time.time()
            
            # 초기 호버링
            self.cmd = ["level", "hover", 0, 0]
            
            self.logger.info("이륙 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"이륙 실패: {e}")
            return False
    
    async def land_and_disarm(self) -> bool:
        """착륙 및 시동 끄기"""
        try:
            self.logger.info("착륙 시작...")
            
            # 호버링
            self.cmd = ["level", "hover", 0, 0]
            await asyncio.sleep(1)
            
            # Offboard 종료
            await self.controller.stop_offboard()
            
            # 착륙
            await self.connection.land()
            self.mission_status.is_flying = False
            await asyncio.sleep(5)
            
            # 시동 끄기
            await self.connection.disarm()
            self.mission_status.is_armed = False
            
            # 비행 기록 저장
            self._save_flight_log()
            
            self.logger.info("착륙 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"착륙 실패: {e}")
            return False
    
    async def emergency_stop(self):
        """긴급 정지"""
        self.logger.warning("!!! 긴급 정지 !!!")
        self.flight_mode = FlightMode.EMERGENCY
        
        # 즉시 정지
        self.cmd = ["level", "hover", 0, 0]
        await self.controller.emergency_stop()
        
        # 큐 비우기
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except:
                pass
    
    async def return_home(self):
        """홈 복귀"""
        self.logger.info("홈 복귀 시작...")
        self.flight_mode = FlightMode.RETURN_HOME
        
        if self.config['home_lat'] and self.config['home_lon']:
            # GPS 홈 위치로 이동
            self.cmd = ["gps", self.config['home_lat'], 
                       self.config['home_lon'], 10.0]
        else:
            # RTL 모드
            await self.controller.return_to_launch()
    
    async def shutdown(self):
        """시스템 종료"""
        self.logger.info("시스템 종료 중...")
        
        self.is_running = False
        
        # 태스크 종료
        tasks = [self.control_task, self.gps_task, 
                self.uart_task, self.monitor_task]
        
        for task in tasks:
            if task:
                task.cancel()
        
        # 통신 종료
        self.uart.stop()
        
        # 연결 해제
        await self.connection.disconnect()
        
        # 로그 저장
        self._save_flight_log()
        
        self.logger.info(f"최종 통계: {self.stats}")
        self.logger.info("시스템 종료 완료")
    
    async def _safety_check(self):
        """안전 체크"""
        try:
            status = await self.connection.get_status()
            
            # 배터리 체크
            if status.battery_percent < self.config['min_battery']:
                self.logger.warning(f"배터리 부족: {status.battery_percent}%")
                
                if status.battery_percent < 15:
                    # 긴급 착륙
                    self.logger.critical("배터리 매우 부족! 긴급 착륙")
                    self.flight_mode = FlightMode.LANDING
                    await self.land_and_disarm()
                elif status.battery_percent < 25:
                    # 홈 복귀
                    self.logger.warning("배터리 부족! 홈 복귀")
                    await self.return_home()
            
            # 고도 체크
            if status.altitude > self.config['max_altitude']:
                self.logger.warning(f"고도 제한 초과: {status.altitude}m")
                self.cmd = ["down", "hover", 0, 30]
            
            # GPS 체크
            if status.gps_satellites < 6:
                self.logger.warning(f"GPS 신호 약함: {status.gps_satellites}개")
                
                if status.gps_satellites < 4:
                    # GPS 없음 - 수동 모드
                    self.flight_mode = FlightMode.MANUAL
            
        except Exception as e:
            self.logger.error(f"안전 체크 오류: {e}")
    
    async def _on_emergency(self, emergency_type: str):
        """긴급 상황 콜백"""
        self.logger.critical(f"긴급 상황: {emergency_type}")
        
        if emergency_type == "low_battery":
            await self.return_home()
        elif emergency_type == "connection_lost":
            await self.emergency_stop()
        elif emergency_type == "geofence_breach":
            await self.return_home()
        else:
            await self.emergency_stop()
    
    def _on_control_command(self, data: Dict):
        """제어 명령 콜백"""
        command = data.get('command', {})
        self.add_command(
            vertical=command.get('vertical', 'level'),
            horizontal=command.get('horizontal', 'hover'),
            rotation=command.get('rotation', 0),
            speed=command.get('speed', 0),
            priority=3,
            source='uart_command'
        )
    
    def _on_gps_command(self, data: Dict):
        """GPS 명령 콜백"""
        command = data.get('command', {})
        if 'latitude' in command and 'longitude' in command:
            if 'altitude' in command:
                self.cmd = ["gps", command['latitude'], 
                          command['longitude'], command['altitude']]
            else:
                self.cmd = ["gps", command['latitude'], 
                          command['longitude']]
    
    def _on_config_update(self, config: Dict):
        """설정 업데이트 콜백"""
        self.logger.info(f"설정 업데이트: {config}")
        self.config.update(config)
        
        # 안전 제한 업데이트
        if self.controller:
            self.controller.set_safety_limits(
                max_altitude=self.config.get('max_altitude', 50.0),
                max_distance=self.config.get('max_distance', 200.0),
                min_battery=self.config.get('min_battery', 20.0)
            )
    
    def _update_mission_status(self, telemetry: Dict):
        """미션 상태 업데이트"""
        self.mission_status.mode = self.flight_mode.value
        self.mission_status.altitude = telemetry.get('position', {}).get('alt_rel', 0.0)
        self.mission_status.battery_percent = telemetry.get('battery', {}).get('percent', 0.0)
        self.mission_status.gps_satellites = telemetry.get('gps', {}).get('satellites', 0)
        self.mission_status.flight_time = self.stats['flight_time']
        self.mission_status.distance_traveled = self.stats['distance_traveled']
        self.mission_status.current_waypoint = self.current_waypoint_idx
        self.mission_status.total_waypoints = len(self.waypoints)
        self.mission_status.errors = self.stats['errors']
    
    async def _send_status(self):
        """상태 전송"""
        status_data = {
            'mission': asdict(self.mission_status),
            'stats': self.stats,
            'current_cmd': self.cmd,
            'flight_mode': self.flight_mode.value,
            'is_initialized': self.is_initialized
        }
        
        self.uart.send_status(status_data)
    
    def _calculate_distance(self, lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
        """GPS 좌표 간 거리 계산"""
        from math import radians, sin, cos, sqrt, atan2
        
        R = 6371000  # 지구 반경 (m)
        
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        
        return R * c
    
    def _log_flight_data(self, event_type: str, data: Dict):
        """비행 데이터 기록"""
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'event': event_type,
            'data': data,
            'flight_time': self.stats['flight_time'],
            'mode': self.flight_mode.value
        }
        
        self.flight_log.append(log_entry)
    
    def _save_flight_log(self):
        """비행 로그 저장"""
        if not self.flight_log:
            return
        
        try:
            filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            
            with open(filename, 'w') as f:
                json.dump({
                    'mission': asdict(self.mission_status),
                    'stats': self.stats,
                    'config': self.config,
                    'log': self.flight_log
                }, f, indent=2)
            
            self.logger.info(f"비행 로그 저장: {filename}")
            
        except Exception as e:
            self.logger.error(f"로그 저장 실패: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """현재 상태 반환"""
        return {
            'mode': self.flight_mode.value,
            'mission': asdict(self.mission_status),
            'current_cmd': self.cmd,
            'stats': self.stats,
            'is_initialized': self.is_initialized,
            'tracking': self.tracking_target is not None,
            'waypoints': len(self.waypoints)
        }


# 메인 실행
async def main():
    """메인 실행"""
    
    # 드론 생성
    drone = Drone()
    
    # 시그널 핸들러
    def signal_handler(sig, frame):
        print("\n종료 신호...")
        asyncio.create_task(drone.emergency_stop())
        asyncio.create_task(drone.shutdown())
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # 초기화
    if not await drone.initialize():
        print("초기화 실패!")
        return
    
    try:
        # 메뉴 표시
        print("\n" + "="*60)
        print("드론 제어 시스템")
        print("="*60)
        print("1. 수동 비행")
        print("2. 웨이포인트 미션")
        print("3. 순찰 모드")
        print("4. 객체 추적 테스트")
        print("5. GPS 테스트")
        print("0. 종료")
        
        choice = input("\n선택: ").strip()
        
        if choice == "1":
            # 수동 비행
            await manual_flight(drone)
            
        elif choice == "2":
            # 웨이포인트 미션
            await waypoint_mission(drone)
            
        elif choice == "3":
            # 순찰 모드
            await patrol_mission(drone)
            
        elif choice == "4":
            # 객체 추적
            await object_tracking_test(drone)
            
        elif choice == "5":
            # GPS 테스트
            await gps_test(drone)
            
    except Exception as e:
        print(f"오류: {e}")
        await drone.emergency_stop()
        
    finally:
        await drone.shutdown()


async def manual_flight(drone: Drone):
    """수동 비행"""
    print("\n=== 수동 비행 모드 ===")
    
    # 이륙
    await drone.arm_and_takeoff(3.0)
    
    # 수동 제어
    drone.set_flight_mode(FlightMode.MANUAL)
    
    commands = [
        ["level", "forward", 0, 50],
        ["up", "forward_right", 45, 60],
        ["level", "hover", 90, 0],
        ["down", "backward", 0, 40],
        ["level", "hover", 0, 0]
    ]
    
    for cmd in commands:
        print(f"실행: {cmd}")
        drone.cmd = cmd
        await asyncio.sleep(3)
    
    # 착륙
    await drone.land_and_disarm()


async def waypoint_mission(drone: Drone):
    """웨이포인트 미션"""
    print("\n=== 웨이포인트 미션 ===")
    
    # 웨이포인트 추가
    drone.add_waypoint(35.123456, 129.123456, 10.0, hold_time=2.0)
    drone.add_waypoint(35.123556, 129.123556, 15.0, action="rotate_360")
    drone.add_waypoint(35.123656, 129.123656, 10.0)
    drone.add_waypoint(35.123456, 129.123456, 5.0)
    
    # 이륙
    await drone.arm_and_takeoff(5.0)
    
    # 웨이포인트 모드
    drone.set_flight_mode(FlightMode.GPS_WAYPOINT)
    
    # 완료 대기
    while drone.flight_mode == FlightMode.GPS_WAYPOINT:
        status = drone.get_status()
        print(f"진행: {status['mission']['current_waypoint']}/{status['mission']['total_waypoints']}")
        await asyncio.sleep(5)
    
    # 착륙
    await drone.land_and_disarm()


async def patrol_mission(drone: Drone):
    """순찰 미션"""
    print("\n=== 순찰 모드 ===")
    
    # 순찰 경로
    patrol_points = [
        (35.123456, 129.123456, 10.0),
        (35.123556, 129.123456, 10.0),
        (35.123556, 129.123556, 10.0),
        (35.123456, 129.123556, 10.0)
    ]
    
    for lat, lon, alt in patrol_points:
        drone.add_waypoint(lat, lon, alt, hold_time=1.0)
    
    # 이륙
    await drone.arm_and_takeoff(5.0)
    
    # 순찰 모드
    drone.set_flight_mode(FlightMode.PATROL)
    
    # 3회 순찰
    for i in range(3):
        print(f"\n순찰 {i+1}/3")
        await asyncio.sleep(60)  # 1분
    
    # 착륙
    drone.set_flight_mode(FlightMode.MANUAL)
    await drone.land_and_disarm()


async def object_tracking_test(drone: Drone):
    """객체 추적 테스트"""
    print("\n=== 객체 추적 테스트 ===")
    
    # 이륙
    await drone.arm_and_takeoff(3.0)
    
    # 가상 객체 생성
    for i in range(10):
        target = ObjectTarget(
            class_name="person",
            confidence=0.95,
            center_x=320 + i * 20,
            center_y=240,
            width=100,
            height=200,
            distance=2.5
        )
        
        drone.set_tracking_target(target)
        await asyncio.sleep(2)
    
    # 착륙
    drone.set_flight_mode(FlightMode.MANUAL)
    await drone.land_and_disarm()


async def gps_test(drone: Drone):
    """GPS 테스트"""
    print("\n=== GPS 테스트 ===")
    
    # 이륙
    await drone.arm_and_takeoff(5.0)
    
    # GPS 좌표
    test_points = [
        ["gps", 35.123456, 129.123456, 10.0],
        ["gps", 35.123556, 129.123556],
        ["gps", 35.123456, 129.123456, 5.0]
    ]
    
    for point in test_points:
        print(f"이동: {point}")
        drone.cmd = point
        await asyncio.sleep(15)
    
    # 착륙
    await drone.land_and_disarm()


# 외부 모듈 연동
def external_control_example():
    """외부 모듈에서 제어"""
    
    async def control():
        drone = Drone()
        await drone.initialize()
        await drone.arm_and_takeoff(3.0)
        
        # 직접 제어
        drone.cmd = ["up", "forward_left", 30, 50]
        await asyncio.sleep(3)
        
        # 객체 추적
        target = ObjectTarget(
            class_name="car",
            confidence=0.88,
            center_x=400,
            center_y=300,
            width=150,
            height=100,
            distance=5.0
        )
        drone.set_tracking_target(target)
        
        await asyncio.sleep(10)
        await drone.land_and_disarm()
        await drone.shutdown()
    
    asyncio.run(control())


if __name__ == "__main__":
    asyncio.run(main())
