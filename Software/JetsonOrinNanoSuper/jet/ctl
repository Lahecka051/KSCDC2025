"""
drone_control.py
H743v2 FC 드론의 방향 제어 모듈
4가지 파라미터로 간단한 제어
"""

import asyncio
import logging
import math
from typing import List, Tuple, Optional, Dict, Any
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

class DroneConnection:
    """드론 FC 연결 관리"""
    
    def __init__(self, connection_string="serial:///dev/ttyTHS1:115200"):
        """
        초기화
        
        Args:
            connection_string (str): FC 연결 문자열
        """
        self.drone = System()
        self.connection_string = connection_string
        self.is_connected = False
        self.logger = logging.getLogger(__name__)
        
    async def connect(self, timeout=30.0) -> bool:
        """FC 연결"""
        self.logger.info(f"FC 연결 시도: {self.connection_string}")
        
        try:
            await self.drone.connect(system_address=self.connection_string)
            
            # 연결 확인
            await asyncio.wait_for(self._wait_for_connection(), timeout=timeout)
            
            # 텔레메트리 확인
            await self._verify_telemetry()
            
            self.is_connected = True
            self.logger.info("FC 연결 성공!")
            return True
            
        except asyncio.TimeoutError:
            self.logger.error("FC 연결 타임아웃!")
            return False
            
        except Exception as e:
            self.logger.error(f"FC 연결 실패: {e}")
            return False
    
    async def _wait_for_connection(self):
        """연결 대기"""
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.logger.info("FC 발견됨")
                return True
    
    async def _verify_telemetry(self):
        """텔레메트리 확인"""
        async for is_armed in self.drone.telemetry.armed():
            self.logger.info(f"텔레메트리 수신 - 시동: {'ON' if is_armed else 'OFF'}")
            break
    
    async def arm(self) -> bool:
        """시동"""
        try:
            await self.drone.action.arm()
            self.logger.info("시동 완료")
            return True
        except Exception as e:
            self.logger.error(f"시동 실패: {e}")
            return False
    
    async def disarm(self) -> bool:
        """시동 끄기"""
        try:
            await self.drone.action.disarm()
            self.logger.info("시동 꺼짐")
            return True
        except Exception as e:
            self.logger.error(f"시동 끄기 실패: {e}")
            return False
    
    async def takeoff(self, altitude=2.0) -> bool:
        """이륙"""
        try:
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            self.logger.info(f"이륙 중... (목표 고도: {altitude}m)")
            return True
        except Exception as e:
            self.logger.error(f"이륙 실패: {e}")
            return False
    
    async def land(self) -> bool:
        """착륙"""
        try:
            await self.drone.action.land()
            self.logger.info("착륙 중...")
            return True
        except Exception as e:
            self.logger.error(f"착륙 실패: {e}")
            return False
    
    async def get_telemetry(self) -> Dict[str, Any]:
        """텔레메트리 데이터 수집"""
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
                    'down': velocity.down_m_s
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
                    'percent': battery.remaining_percent
                }
                break
            
            # GPS 상태
            async for gps in self.drone.telemetry.gps_info():
                telemetry['gps'] = {
                    'satellites': gps.num_satellites,
                    'fix_type': gps.fix_type.value
                }
                break
                
        except Exception as e:
            self.logger.error(f"텔레메트리 수집 오류: {e}")
        
        return telemetry


class DroneController:
    """드론 방향 제어 - 간소화된 인터페이스"""
    
    def __init__(self, connection: DroneConnection):
        """
        초기화
        
        Args:
            connection (DroneConnection): 드론 연결 객체
        """
        self.connection = connection
        self.drone = connection.drone
        self.logger = logging.getLogger(__name__)
        
        # Offboard 모드 상태
        self.offboard_enabled = False
        
        # 속도 제한 설정
        self.MAX_SPEED_HORIZONTAL = 10.0  # m/s (100% 모터 부하 시)
        self.MAX_SPEED_VERTICAL = 3.0     # m/s (100% 모터 부하 시)
        self.MAX_ROTATION_SPEED = 60.0    # deg/s (최대 회전 속도)
        
    async def start_offboard(self) -> bool:
        """Offboard 모드 시작"""
        try:
            # 초기 설정값
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            
            # Offboard 시작
            await self.drone.offboard.start()
            self.offboard_enabled = True
            self.logger.info("Offboard 모드 활성화")
            return True
            
        except Exception as e:
            self.logger.error(f"Offboard 모드 시작 실패: {e}")
            return False
    
    async def stop_offboard(self) -> bool:
        """Offboard 모드 종료"""
        try:
            await self.drone.offboard.stop()
            self.offboard_enabled = False
            self.logger.info("Offboard 모드 종료")
            return True
            
        except Exception as e:
            self.logger.error(f"Offboard 모드 종료 실패: {e}")
            return False
    
    async def execute_cmd(self, cmd: List) -> bool:
        """
        드론 제어 명령 실행
        
        Args:
            cmd: 
                - 4개 값: [vertical, horizontal, rotation, motor_percent]
                - 3개 값 (GPS): ["gps", latitude, longitude, altitude(선택)]
                - 4개 값 (GPS): ["gps", latitude, longitude, altitude]
        
        Examples:
            drone.cmd = ["up", "forward_left", 30, 50]  # 일반 제어
            drone.cmd = ["gps", 35.123456, 129.123456]  # GPS 이동 (현재 고도 유지)
            drone.cmd = ["gps", 35.123456, 129.123456, 10.0]  # GPS 이동 (지정 고도)
        """
        if len(cmd) < 3 or len(cmd) > 4:
            self.logger.error(f"잘못된 명령 형식: {cmd}")
            return False
        
        # GPS 명령 처리
        if cmd[0] == "gps":
            return await self._execute_gps_cmd(cmd)
        
        # 일반 제어 명령 처리
        if len(cmd) != 4:
            self.logger.error(f"일반 제어는 4개 값 필요: {cmd}")
            return False
        
        vertical, horizontal, rotation, motor_percent = cmd
        
        if not self.connection.is_connected:
            self.logger.warning("드론이 연결되지 않음")
            return False
        
        try:
            # 모터 부하를 속도로 변환 (0-100% -> 0-MAX_SPEED m/s)
            h_speed = (motor_percent / 100.0) * self.MAX_SPEED_HORIZONTAL
            v_speed = (motor_percent / 100.0) * self.MAX_SPEED_VERTICAL
            
            # NED 속도 계산
            north = 0.0  # 북쪽(전진+)
            east = 0.0   # 동쪽(우측+)
            down = 0.0   # 아래(하강+)
            
            # 1. 수직 속도 처리
            if vertical == "up":
                down = -v_speed
            elif vertical == "down":
                down = v_speed
            # "level"인 경우 down = 0
            
            # 2. 수평 속도 처리
            if horizontal == "forward":
                north = h_speed
            elif horizontal == "backward":
                north = -h_speed
            elif horizontal == "left":
                east = -h_speed
            elif horizontal == "right":
                east = h_speed
            elif horizontal == "forward_left":
                north = h_speed * 0.707  # 45도 대각선
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
            # "hover"인 경우 north = east = 0
            
            # 3. 회전 처리 (0-359도를 -180~180도로 변환)
            yaw_rate = 0.0
            if rotation != 0:
                # 최단 경로 회전
                if rotation > 180:
                    yaw_rate = -(360 - rotation)
                else:
                    yaw_rate = rotation
                
                # 회전 속도 제한
                yaw_rate = max(-self.MAX_ROTATION_SPEED, 
                              min(self.MAX_ROTATION_SPEED, yaw_rate))
            
            # 4. 명령 전송
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(north, east, down, yaw_rate)
            )
            
            self.logger.info(f"CMD 실행: [{vertical}, {horizontal}, {rotation}°, {motor_percent}%] "
                           f"-> NED({north:.1f}, {east:.1f}, {down:.1f}) Yaw:{yaw_rate:.1f}°/s")
            
            return True
            
        except Exception as e:
            self.logger.error(f"명령 실행 실패: {e}")
            return False
    
    async def _execute_gps_cmd(self, cmd: List) -> bool:
        """
        GPS 좌표로 이동
        
        Args:
            cmd: ["gps", latitude, longitude, altitude(선택)]
        """
        try:
            if len(cmd) == 3:
                # 현재 고도 유지
                lat, lon = cmd[1], cmd[2]
                
                # 현재 고도 가져오기
                current_alt = 10.0  # 기본값
                async for position in self.drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    break
                
                alt = current_alt
                
            elif len(cmd) == 4:
                # 지정 고도
                lat, lon, alt = cmd[1], cmd[2], cmd[3]
            
            else:
                self.logger.error(f"GPS 명령 형식 오류: {cmd}")
                return False
            
            # 현재 heading 유지
            current_heading = 0.0
            async for attitude in self.drone.telemetry.attitude_euler():
                current_heading = attitude.yaw_deg
                break
            
            # GPS 좌표로 이동
            await self.drone.action.goto_location(
                latitude_deg=lat,
                longitude_deg=lon,
                absolute_altitude_m=alt,
                yaw_deg=current_heading
            )
            
            self.logger.info(f"GPS 이동: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            return True
            
        except Exception as e:
            self.logger.error(f"GPS 이동 실패: {e}")
            return False
    
    async def stop(self) -> bool:
        """정지 (호버링)"""
        return await self.execute_cmd(["level", "hover", 0, 0])
    
    async def emergency_stop(self) -> bool:
        """긴급 정지"""
        try:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            await self.drone.action.hold()
            self.logger.warning("긴급 정지 실행!")
            return True
            
        except Exception as e:
            self.logger.error(f"긴급 정지 실패: {e}")
            return False


# 간단한 사용 예제
async def simple_example():
    """간단한 사용 예제"""
    
    # 연결
    connection = DroneConnection("serial:///dev/ttyTHS1:115200")
    if not await connection.connect():
        print("연결 실패!")
        return
    
    controller = DroneController(connection)
    
    # 시동 및 이륙
    await connection.arm()
    await asyncio.sleep(2)
    await connection.takeoff(2.0)
    await asyncio.sleep(5)
    
    # Offboard 모드 시작
    await controller.start_offboard()
    
    # 간단한 4개 파라미터 명령
    commands = [
        ["level", "forward", 0, 50],       # 전진 50%
        ["up", "forward_right", 0, 60],    # 우상향 대각선 60%
        ["level", "hover", 90, 0],         # 90도 회전
        ["down", "backward", 0, 40],       # 후진하며 하강 40%
        ["level", "hover", 0, 0],          # 정지
    ]
    
    for cmd in commands:
        print(f"실행: {cmd}")
        await controller.execute_cmd(cmd)
        await asyncio.sleep(3)
    
    # Offboard 모드 종료
    await controller.stop_offboard()
    
    # 착륙
    await connection.land()
    await asyncio.sleep(5)
    await connection.disarm()
    
    print("완료!")


if __name__ == "__main__":
    asyncio.run(simple_example())
