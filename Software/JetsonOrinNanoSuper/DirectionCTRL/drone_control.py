"""
drone_control.py - FC 직접 연결 버전
H743v2 FC와 Jetson이 /dev/ttyTHS1:115200으로 직접 연결
MAVSDK를 통한 MAVLink 통신
"""

import asyncio
import logging
import math
import time
from typing import List, Tuple, Optional, Dict, Any, Callable
from enum import Enum
from dataclasses import dataclass
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, VelocityBodyYawspeed
from mavsdk.action import ActionError
from mavsdk.telemetry import FlightMode

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

class ControlMode(Enum):
    """제어 모드"""
    MANUAL = "manual"
    GUIDED = "guided"
    OFFBOARD = "offboard"
    POSITION = "position"
    AUTO = "auto"
    LAND = "land"
    RTL = "rtl"

@dataclass
class FlightStatus:
    """비행 상태"""
    is_armed: bool = False
    is_flying: bool = False
    mode: str = "UNKNOWN"
    altitude: float = 0.0
    battery_percent: float = 0.0
    gps_satellites: int = 0
    heading: float = 0.0

@dataclass
class SafetyLimits:
    """안전 제한"""
    max_altitude: float = 50.0      # 최대 고도 (m)
    max_distance: float = 100.0     # 최대 거리 (m)
    min_battery: float = 20.0       # 최소 배터리 (%)
    max_speed_horizontal: float = 10.0  # 최대 수평 속도 (m/s)
    max_speed_vertical: float = 3.0     # 최대 수직 속도 (m/s)
    max_rotation_speed: float = 60.0    # 최대 회전 속도 (deg/s)
    geofence_radius: float = 150.0      # 지오펜스 반경 (m)

class DroneController:
    """드론 FC 직접 제어 - /dev/ttyTHS1 사용"""
    
    def __init__(self, connection_string="serial:///dev/ttyTHS1:115200"):
        """
        초기화
        
        Args:
            connection_string (str): FC 연결 문자열 (기본: /dev/ttyTHS1)
        """
        self.drone = System()
        self.connection_string = connection_string
        self.is_connected = False
        self.logger = logging.getLogger(__name__)
        
        # 제어 모드
        self.control_mode = ControlMode.MANUAL
        self.offboard_enabled = False
        
        # 안전 제한
        self.safety = SafetyLimits()
        
        # Home 위치
        self.home_position = None
        
        # 제어 상태
        self.last_command_time = time.time()
        self.command_timeout = 1.0  # 1초
        
        # 텔레메트리 캐시
        self.telemetry_cache = {}
        self.cache_timeout = 0.5  # 500ms
        
        # 콜백
        self.emergency_callback = None
        
        # 연결 재시도 설정
        self.max_retries = 3
        self.retry_delay = 2.0
        
    async def connect(self, timeout=30.0) -> bool:
        """FC 연결 (재시도 포함)"""
        for attempt in range(self.max_retries):
            try:
                self.logger.info(f"FC 연결 시도 {attempt+1}/{self.max_retries}: {self.connection_string}")
                
                # MAVLink 연결
                await self.drone.connect(system_address=self.connection_string)
                
                # 연결 확인
                connected = await self._wait_for_connection(timeout)
                
                if connected:
                    # 텔레메트리 확인
                    await self._verify_telemetry()
                    
                    self.is_connected = True
                    self.logger.info("FC 연결 성공!")
                    
                    # 초기 설정
                    await self._configure_drone()
                    
                    return True
                
            except asyncio.TimeoutError:
                self.logger.warning(f"연결 시도 {attempt+1} 타임아웃")
                
            except Exception as e:
                self.logger.error(f"연결 시도 {attempt+1} 실패: {e}")
            
            if attempt < self.max_retries - 1:
                await asyncio.sleep(self.retry_delay)
        
        self.logger.error("FC 연결 최종 실패!")
        return False
    
    async def _wait_for_connection(self, timeout):
        """연결 대기"""
        try:
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("FC 발견됨")
                    return True
        except:
            return False
    
    async def _verify_telemetry(self):
        """텔레메트리 확인"""
        try:
            async for is_armed in self.drone.telemetry.armed():
                self.logger.info(f"텔레메트리 수신 - 시동: {'ON' if is_armed else 'OFF'}")
                break
        except:
            pass
    
    async def _configure_drone(self):
        """드론 초기 설정"""
        try:
            # 파라미터 설정
            await self.drone.param.set_param_float("MIS_TAKEOFF_ALT", 2.0)
            await self.drone.param.set_param_float("RTL_RETURN_ALT", 20.0)
            await self.drone.param.set_param_float("LAND_SPEED", 0.5)
            
            self.logger.info("드론 파라미터 설정 완료")
        except:
            self.logger.warning("일부 파라미터 설정 실패")
    
    async def disconnect(self):
        """연결 해제"""
        self.is_connected = False
        self.logger.info("FC 연결 해제")
    
    async def arm(self) -> bool:
        """시동 (안전 체크 포함)"""
        try:
            # 안전 체크
            if not await self._pre_arm_check():
                return False
            
            self.logger.info("시동 중...")
            await self.drone.action.arm()
            
            # 시동 확인
            for _ in range(10):
                async for is_armed in self.drone.telemetry.armed():
                    if is_armed:
                        self.logger.info("시동 완료!")
                        return True
                    break
                await asyncio.sleep(0.5)
            
            self.logger.error("시동 확인 실패")
            return False
            
        except ActionError as e:
            self.logger.error(f"시동 실패: {e}")
            return False
    
    async def _pre_arm_check(self) -> bool:
        """시동 전 안전 체크"""
        try:
            # 배터리 체크
            async for battery in self.drone.telemetry.battery():
                if battery.remaining_percent < 30:
                    self.logger.warning(f"배터리 부족: {battery.remaining_percent}%")
                    return False
                break
            
            # GPS 체크
            async for gps in self.drone.telemetry.gps_info():
                if gps.num_satellites < 6:
                    self.logger.warning(f"GPS 위성 부족: {gps.num_satellites}개")
                    return False
                break
            
            # 캘리브레이션 체크
            async for health in self.drone.telemetry.health():
                if not health.is_accelerometer_calibration_ok:
                    self.logger.warning("가속도계 캘리브레이션 필요")
                    return False
                if not health.is_magnetometer_calibration_ok:
                    self.logger.warning("지자기계 캘리브레이션 필요")
                    return False
                break
            
            return True
            
        except Exception as e:
            self.logger.error(f"안전 체크 실패: {e}")
            return False
    
    async def disarm(self, force=False) -> bool:
        """시동 끄기"""
        try:
            if force:
                self.logger.warning("강제 시동 끄기")
                await self.drone.action.kill()
            else:
                self.logger.info("시동 끄기...")
                await self.drone.action.disarm()
            
            self.logger.info("시동 꺼짐")
            return True
            
        except ActionError as e:
            self.logger.error(f"시동 끄기 실패: {e}")
            return False
    
    async def takeoff(self, altitude=2.0) -> bool:
        """이륙"""
        try:
            # 고도 제한 체크
            if altitude > self.safety.max_altitude:
                self.logger.warning(f"고도 제한: {altitude}m -> {self.safety.max_altitude}m")
                altitude = self.safety.max_altitude
            
            self.logger.info(f"이륙 중... (목표 고도: {altitude}m)")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            
            # 이륙 완료 대기
            target_alt = altitude * 0.95
            for _ in range(30):  # 최대 15초 대기
                async for position in self.drone.telemetry.position():
                    if position.relative_altitude_m >= target_alt:
                        self.logger.info(f"이륙 완료: {position.relative_altitude_m:.1f}m")
                        return True
                    break
                await asyncio.sleep(0.5)
            
            self.logger.warning("이륙 타임아웃")
            return False
            
        except ActionError as e:
            self.logger.error(f"이륙 실패: {e}")
            return False
    
    async def land(self) -> bool:
        """착륙"""
        try:
            self.logger.info("착륙 중...")
            await self.drone.action.land()
            
            # 착륙 완료 대기
            for _ in range(60):  # 최대 30초 대기
                async for in_air in self.drone.telemetry.in_air():
                    if not in_air:
                        self.logger.info("착륙 완료!")
                        return True
                    break
                await asyncio.sleep(0.5)
            
            self.logger.warning("착륙 타임아웃")
            return False
            
        except ActionError as e:
            self.logger.error(f"착륙 실패: {e}")
            return False
    
    async def start_offboard(self) -> bool:
        """Offboard/Guided 모드 시작"""
        try:
            if self.offboard_enabled:
                return True
            
            self.logger.info("Offboard 모드 시작...")
            
            # 초기 설정값
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            
            # Offboard 시작
            await self.drone.offboard.start()
            
            self.offboard_enabled = True
            self.control_mode = ControlMode.OFFBOARD
            self.logger.info("Offboard 모드 활성화")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Offboard 모드 시작 실패: {e}")
            return False
    
    async def stop_offboard(self) -> bool:
        """Offboard 모드 종료"""
        try:
            if not self.offboard_enabled:
                return True
            
            await self.drone.offboard.stop()
            self.offboard_enabled = False
            self.control_mode = ControlMode.MANUAL
            self.logger.info("Offboard 모드 종료")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Offboard 모드 종료 실패: {e}")
            return False
    
    async def execute_cmd(self, cmd: List) -> bool:
        """
        드론 제어 명령 실행
        
        Args:
            cmd: 제어 명령
                - 일반: [vertical, horizontal, rotation, motor_percent]
                - GPS: ["gps", latitude, longitude, altitude(선택)]
        """
        try:
            # 안전 체크
            if not await self._safety_check():
                return False
            
            # 명령 타임아웃 체크
            self.last_command_time = time.time()
            
            # GPS 명령 처리
            if cmd[0] == "gps":
                return await self._execute_gps_cmd(cmd)
            
            # 일반 제어 명령 처리
            if len(cmd) != 4:
                self.logger.error(f"잘못된 명령 형식: {cmd}")
                return False
            
            vertical, horizontal, rotation, motor_percent = cmd
            
            # 모터 퍼센트를 속도로 변환
            h_speed = (motor_percent / 100.0) * self.safety.max_speed_horizontal
            v_speed = (motor_percent / 100.0) * self.safety.max_speed_vertical
            
            # NED 속도 계산
            north, east, down = self._calculate_ned_velocity(
                vertical, horizontal, h_speed, v_speed
            )
            
            # 회전 속도 계산
            yaw_rate = self._calculate_yaw_rate(rotation)
            
            # 속도 제한 적용
            north = self._limit_speed(north, self.safety.max_speed_horizontal)
            east = self._limit_speed(east, self.safety.max_speed_horizontal)
            down = self._limit_speed(down, self.safety.max_speed_vertical)
            yaw_rate = self._limit_speed(yaw_rate, self.safety.max_rotation_speed)
            
            # 명령 전송
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north, east, down, yaw_rate)
            )
            
            # Offboard 모드 확인
            if not self.offboard_enabled:
                await self.start_offboard()
            
            self.logger.debug(f"CMD: {cmd} -> NED({north:.1f}, {east:.1f}, {down:.1f}) Yaw:{yaw_rate:.1f}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"명령 실행 실패: {e}")
            return False
    
    def _calculate_ned_velocity(self, vertical: str, horizontal: str, 
                                h_speed: float, v_speed: float) -> Tuple[float, float, float]:
        """NED 속도 계산"""
        north = 0.0
        east = 0.0
        down = 0.0
        
        # 수직 속도
        if vertical == "up":
            down = -v_speed
        elif vertical == "down":
            down = v_speed
        
        # 수평 속도
        if horizontal == "forward":
            north = h_speed
        elif horizontal == "backward":
            north = -h_speed
        elif horizontal == "left":
            east = -h_speed
        elif horizontal == "right":
            east = h_speed
        elif horizontal == "forward_left":
            north = h_speed * 0.707
            east = -h_speed * 0.707
        elif horizontal == "forward_right":
            north = h_speed * 0.707
            east = h_speed * 0.707
        elif horizontal == "backward_left":
            north = -h_speed * 0.707
            east = -h_speed * 0.707
        elif horizontal == "backward_right":
            north = -h_speed * 0.707
            east = h_speed * 0.707
        
        return north, east, down
    
    def _calculate_yaw_rate(self, rotation: float) -> float:
        """Yaw 회전 속도 계산"""
        if rotation == 0:
            return 0.0
        
        # 최단 경로 회전
        if rotation > 180:
            yaw_rate = -(360 - rotation) * 0.5
        else:
            yaw_rate = rotation * 0.5
        
        return yaw_rate
    
    def _limit_speed(self, speed: float, max_speed: float) -> float:
        """속도 제한"""
        return max(-max_speed, min(max_speed, speed))
    
    async def _execute_gps_cmd(self, cmd: List) -> bool:
        """GPS 좌표로 이동"""
        try:
            lat = float(cmd[1])
            lon = float(cmd[2])
            
            # 고도 처리
            if len(cmd) >= 4:
                alt = float(cmd[3])
            else:
                # 현재 고도 유지
                telemetry = await self.get_telemetry()
                alt = telemetry.get('position', {}).get('alt_rel', 10.0)
            
            # 거리 체크
            if self.home_position:
                distance = self._calculate_distance(
                    self.home_position[0], self.home_position[1],
                    lat, lon
                )
                
                if distance > self.safety.max_distance:
                    self.logger.warning(f"최대 거리 초과: {distance:.1f}m")
                    return False
            
            # 현재 heading 유지
            current_heading = 0.0
            async for attitude in self.drone.telemetry.attitude_euler():
                current_heading = attitude.yaw_deg
                break
            
            # GPS 이동
            await self.drone.action.goto_location(lat, lon, alt, current_heading)
            
            self.logger.info(f"GPS 이동: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            return True
            
        except Exception as e:
            self.logger.error(f"GPS 이동 실패: {e}")
            return False
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                           lat2: float, lon2: float) -> float:
        """두 GPS 좌표 간 거리 계산"""
        R = 6371000  # 지구 반경 (m)
        
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    async def _safety_check(self) -> bool:
        """안전 체크"""
        try:
            status = await self.get_status()
            
            # 배터리 체크
            if status.battery_percent < self.safety.min_battery:
                self.logger.warning(f"배터리 부족: {status.battery_percent}%")
                if self.emergency_callback:
                    await self.emergency_callback("low_battery")
                return False
            
            # 고도 체크
            if status.altitude > self.safety.max_altitude:
                self.logger.warning(f"고도 제한 초과: {status.altitude}m")
                return False
            
            # GPS 체크
            if status.gps_satellites < 6:
                self.logger.warning(f"GPS 신호 약함: {status.gps_satellites}개")
            
            return True
            
        except:
            return True  # 체크 실패 시 계속 진행
    
    async def get_telemetry(self) -> Dict[str, Any]:
        """텔레메트리 데이터 수집 (캐시 사용)"""
        current_time = time.time()
        
        # 캐시 확인
        if self.telemetry_cache.get('timestamp', 0) + self.cache_timeout > current_time:
            return self.telemetry_cache.get('data', {})
        
        telemetry = {}
        
        try:
            # 위치
            async for position in self.drone.telemetry.position():
                telemetry['position'] = {
                    'lat': position.latitude_deg,
                    'lon': position.longitude_deg,
                    'alt_abs': position.absolute_altitude_m,
                    'alt_rel': position.relative_altitude_m
                }
                break
            
            # 속도
            async for velocity in self.drone.telemetry.velocity_ned():
                telemetry['velocity'] = {
                    'north': velocity.north_m_s,
                    'east': velocity.east_m_s,
                    'down': velocity.down_m_s,
                    'ground_speed': math.sqrt(velocity.north_m_s**2 + velocity.east_m_s**2)
                }
                break
            
            # 자세
            async for attitude in self.drone.telemetry.attitude_euler():
                telemetry['attitude'] = {
                    'roll': attitude.roll_deg,
                    'pitch': attitude.pitch_deg,
                    'yaw': attitude.yaw_deg
                }
                break
            
            # 배터리
            async for battery in self.drone.telemetry.battery():
                telemetry['battery'] = {
                    'voltage': battery.voltage_v,
                    'current': battery.current_battery_a,
                    'percent': battery.remaining_percent
                }
                break
            
            # GPS
            async for gps in self.drone.telemetry.gps_info():
                telemetry['gps'] = {
                    'satellites': gps.num_satellites,
                    'fix_type': gps.fix_type.value
                }
                break
            
            # 상태
            async for armed in self.drone.telemetry.armed():
                telemetry['armed'] = armed
                break
            
            async for in_air in self.drone.telemetry.in_air():
                telemetry['in_air'] = in_air
                break
            
            async for mode in self.drone.telemetry.flight_mode():
                telemetry['flight_mode'] = str(mode)
                break
            
            # 캐시 업데이트
            self.telemetry_cache = {
                'timestamp': current_time,
                'data': telemetry
            }
            
        except Exception as e:
            self.logger.error(f"텔레메트리 수집 오류: {e}")
        
        return telemetry
    
    async def get_status(self) -> FlightStatus:
        """비행 상태 가져오기"""
        status = FlightStatus()
        
        try:
            telemetry = await self.get_telemetry()
            
            status.is_armed = telemetry.get('armed', False)
            status.is_flying = telemetry.get('in_air', False)
            status.mode = telemetry.get('flight_mode', 'UNKNOWN')
            status.altitude = telemetry.get('position', {}).get('alt_rel', 0.0)
            status.battery_percent = telemetry.get('battery', {}).get('percent', 0.0)
            status.gps_satellites = telemetry.get('gps', {}).get('satellites', 0)
            status.heading = telemetry.get('attitude', {}).get('yaw', 0.0)
            
        except:
            pass
        
        return status
    
    async def stop(self) -> bool:
        """정지 (호버링)"""
        return await self.execute_cmd(["level", "hover", 0, 0])
    
    async def emergency_stop(self) -> bool:
        """긴급 정지"""
        try:
            self.logger.warning("긴급 정지!")
            
            # 즉시 정지
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            
            # Hold 모드
            await self.drone.action.hold()
            
            # 콜백 호출
            if self.emergency_callback:
                await self.emergency_callback("emergency_stop")
            
            return True
            
        except Exception as e:
            self.logger.error(f"긴급 정지 실패: {e}")
            return False
    
    async def return_to_launch(self) -> bool:
        """RTL (Return to Launch)"""
        try:
            self.logger.info("RTL 실행...")
            await self.drone.action.return_to_launch()
            self.control_mode = ControlMode.RTL
            return True
            
        except Exception as e:
            self.logger.error(f"RTL 실패: {e}")
            return False
    
    def set_home_position(self, lat: float, lon: float):
        """Home 위치 설정"""
        self.home_position = (lat, lon)
        self.logger.info(f"Home 위치: ({lat:.6f}, {lon:.6f})")
    
    def set_safety_limits(self, **kwargs):
        """안전 제한 설정"""
        for key, value in kwargs.items():
            if hasattr(self.safety, key):
                setattr(self.safety, key, value)
                self.logger.info(f"안전 제한 변경: {key} = {value}")
    
    def set_emergency_callback(self, callback: Callable):
        """긴급 상황 콜백 설정"""
        self.emergency_callback = callback
