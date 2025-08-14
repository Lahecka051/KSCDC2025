"""
ardupilot_guided_control.py
ArduPilot Guided 모드를 통한 H743v2 FC 직접 제어 모듈
Jetson에서 MAVLink를 통해 FC에 직접 명령 전송
"""

import asyncio
import time
import math
import logging
from typing import Optional, Tuple, Dict, Any
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, VelocityBodyYawspeed
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

class GuidedModeController:
    """ArduPilot Guided 모드 컨트롤러"""
    
    def __init__(self, connection_string="serial:///dev/ttyTHS1:115200"):
        """
        초기화
        
        Args:
            connection_string (str): FC 연결 문자열 (UART)
        """
        self.drone = System()
        self.connection_string = connection_string
        self.logger = logging.getLogger(__name__)
        
        # 상태
        self.is_connected = False
        self.is_armed = False
        self.in_guided_mode = False
        
        # 현재 상태 정보
        self.current_position = None
        self.current_heading = 0.0
        self.current_altitude = 0.0
        
        # 제어 파라미터
        self.max_speed = 10.0  # m/s
        self.max_yaw_speed = 60.0  # deg/s
        self.position_tolerance = 2.0  # meters
        
    async def connect(self) -> bool:
        """FC 연결"""
        try:
            self.logger.info(f"FC 연결 시도: {self.connection_string}")
            await self.drone.connect(system_address=self.connection_string)
            
            # 연결 확인
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("FC 연결 성공!")
                    self.is_connected = True
                    break
            
            # 텔레메트리 확인
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok:
                    self.logger.info("GPS 신호 정상")
                    break
            
            return True
            
        except Exception as e:
            self.logger.error(f"FC 연결 실패: {e}")
            return False
    
    async def set_guided_mode(self) -> bool:
        """Guided 모드 설정"""
        try:
            # 현재 모드 확인
            async for flight_mode in self.drone.telemetry.flight_mode():
                current_mode = str(flight_mode)
                self.logger.info(f"현재 모드: {current_mode}")
                break
            
            # Guided 모드로 변경
            self.logger.info("GUIDED 모드로 변경 중...")
            
            # ArduPilot의 경우 모드 변경 명령
            # GUIDED 모드 = MAV_MODE_GUIDED_ARMED
            await self.drone.action.set_return_to_launch_altitude(20.0)  # RTL 고도 설정
            
            # Offboard 모드 시작 (MAVSDK에서는 Offboard = Guided)
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            
            self.in_guided_mode = True
            self.logger.info("GUIDED 모드 활성화!")
            return True
            
        except Exception as e:
            self.logger.error(f"GUIDED 모드 설정 실패: {e}")
            return False
    
    async def arm(self) -> bool:
        """시동"""
        try:
            self.logger.info("시동 중...")
            await self.drone.action.arm()
            self.is_armed = True
            self.logger.info("시동 완료!")
            return True
            
        except ActionError as e:
            self.logger.error(f"시동 실패: {e}")
            return False
    
    async def disarm(self) -> bool:
        """시동 끄기"""
        try:
            self.logger.info("시동 끄기...")
            await self.drone.action.disarm()
            self.is_armed = False
            self.logger.info("시동 꺼짐!")
            return True
            
        except ActionError as e:
            self.logger.error(f"시동 끄기 실패: {e}")
            return False
    
    async def takeoff_guided(self, altitude: float = 5.0) -> bool:
        """Guided 모드 이륙"""
        try:
            if not self.is_armed:
                self.logger.warning("시동이 걸려있지 않습니다!")
                return False
            
            # Guided 모드 확인
            if not self.in_guided_mode:
                await self.set_guided_mode()
            
            self.logger.info(f"GUIDED 이륙: {altitude}m")
            
            # 이륙 명령
            await self.drone.action.takeoff()
            
            # 목표 고도 도달 대기
            while True:
                async for position in self.drone.telemetry.position():
                    current_alt = position.relative_altitude_m
                    if current_alt >= altitude * 0.95:  # 95% 도달
                        self.logger.info(f"목표 고도 도달: {current_alt:.1f}m")
                        return True
                    await asyncio.sleep(0.5)
                    break
                    
        except Exception as e:
            self.logger.error(f"GUIDED 이륙 실패: {e}")
            return False
    
    async def goto_position_ned(self, north: float, east: float, down: float, yaw: float = 0.0) -> bool:
        """
        NED 좌표계 위치로 이동
        
        Args:
            north: 북쪽 방향 거리 (m)
            east: 동쪽 방향 거리 (m)
            down: 아래 방향 거리 (m, 음수는 상승)
            yaw: 방향 (도)
        """
        try:
            self.logger.info(f"NED 이동: N={north:.1f}, E={east:.1f}, D={down:.1f}, Yaw={yaw:.1f}")
            
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(north, east, down, yaw)
            )
            
            # Offboard 모드 시작 (처음만)
            if not self.in_guided_mode:
                await self.drone.offboard.start()
                self.in_guided_mode = True
            
            return True
            
        except Exception as e:
            self.logger.error(f"NED 이동 실패: {e}")
            return False
    
    async def goto_position_global(self, latitude: float, longitude: float, altitude: float) -> bool:
        """
        GPS 좌표로 이동
        
        Args:
            latitude: 위도
            longitude: 경도
            altitude: 고도 (m, 해발고도)
        """
        try:
            self.logger.info(f"GPS 이동: ({latitude:.6f}, {longitude:.6f}, {altitude:.1f}m)")
            
            # 현재 heading 유지
            async for attitude in self.drone.telemetry.attitude_euler():
                self.current_heading = attitude.yaw_deg
                break
            
            # GPS 좌표로 이동
            await self.drone.action.goto_location(
                latitude, longitude, altitude, self.current_heading
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"GPS 이동 실패: {e}")
            return False
    
    async def set_velocity_ned(self, vn: float, ve: float, vd: float, yaw_rate: float = 0.0) -> bool:
        """
        NED 속도 제어
        
        Args:
            vn: 북쪽 속도 (m/s)
            ve: 동쪽 속도 (m/s)
            vd: 아래 속도 (m/s, 음수는 상승)
            yaw_rate: Yaw 회전 속도 (deg/s)
        """
        try:
            # 속도 제한
            vn = max(-self.max_speed, min(self.max_speed, vn))
            ve = max(-self.max_speed, min(self.max_speed, ve))
            vd = max(-self.max_speed/2, min(self.max_speed/2, vd))
            yaw_rate = max(-self.max_yaw_speed, min(self.max_yaw_speed, yaw_rate))
            
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vn, ve, vd, yaw_rate)
            )
            
            # Offboard 모드 시작 (처음만)
            if not self.in_guided_mode:
                await self.drone.offboard.start()
                self.in_guided_mode = True
            
            self.logger.debug(f"속도 설정: VN={vn:.1f}, VE={ve:.1f}, VD={vd:.1f}, Yaw={yaw_rate:.1f}")
            return True
            
        except Exception as e:
            self.logger.error(f"속도 설정 실패: {e}")
            return False
    
    async def set_velocity_body(self, forward: float, right: float, down: float, yaw_rate: float = 0.0) -> bool:
        """
        기체 좌표계 속도 제어
        
        Args:
            forward: 전진 속도 (m/s)
            right: 우측 속도 (m/s)
            down: 하강 속도 (m/s)
            yaw_rate: Yaw 회전 속도 (deg/s)
        """
        try:
            # 속도 제한
            forward = max(-self.max_speed, min(self.max_speed, forward))
            right = max(-self.max_speed, min(self.max_speed, right))
            down = max(-self.max_speed/2, min(self.max_speed/2, down))
            yaw_rate = max(-self.max_yaw_speed, min(self.max_yaw_speed, yaw_rate))
            
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward, right, down, yaw_rate)
            )
            
            # Offboard 모드 시작 (처음만)
            if not self.in_guided_mode:
                await self.drone.offboard.start()
                self.in_guided_mode = True
            
            self.logger.debug(f"기체 속도: F={forward:.1f}, R={right:.1f}, D={down:.1f}, Yaw={yaw_rate:.1f}")
            return True
            
        except Exception as e:
            self.logger.error(f"기체 속도 설정 실패: {e}")
            return False
    
    async def move_relative(self, forward: float, right: float, down: float) -> bool:
        """
        현재 위치 기준 상대 이동
        
        Args:
            forward: 전방 이동 거리 (m)
            right: 우측 이동 거리 (m)
            down: 하강 거리 (m, 음수는 상승)
        """
        try:
            # 현재 heading 가져오기
            async for attitude in self.drone.telemetry.attitude_euler():
                heading_rad = math.radians(attitude.yaw_deg)
                break
            
            # Body to NED 변환
            north = forward * math.cos(heading_rad) - right * math.sin(heading_rad)
            east = forward * math.sin(heading_rad) + right * math.cos(heading_rad)
            
            self.logger.info(f"상대 이동: F={forward:.1f}, R={right:.1f}, D={down:.1f}")
            
            # NED 좌표로 이동
            return await self.goto_position_ned(north, east, down)
            
        except Exception as e:
            self.logger.error(f"상대 이동 실패: {e}")
            return False
    
    async def rotate_yaw(self, angle_deg: float) -> bool:
        """
        Yaw 회전
        
        Args:
            angle_deg: 회전 각도 (양수=시계방향)
        """
        try:
            self.logger.info(f"회전: {angle_deg:.1f}°")
            
            # 회전 속도 계산
            rotation_speed = 30.0 if angle_deg > 0 else -30.0
            duration = abs(angle_deg) / 30.0
            
            # 회전 실행
            await self.set_velocity_ned(0, 0, 0, rotation_speed)
            await asyncio.sleep(duration)
            
            # 정지
            await self.set_velocity_ned(0, 0, 0, 0)
            
            return True
            
        except Exception as e:
            self.logger.error(f"회전 실패: {e}")
            return False
    
    async def land_guided(self) -> bool:
        """Guided 모드 착륙"""
        try:
            self.logger.info("GUIDED 착륙 시작...")
            await self.drone.action.land()
            
            # 착륙 완료 대기
            while True:
                async for in_air in self.drone.telemetry.in_air():
                    if not in_air:
                        self.logger.info("착륙 완료!")
                        return True
                    await asyncio.sleep(0.5)
                    break
                    
        except Exception as e:
            self.logger.error(f"착륙 실패: {e}")
            return False
    
    async def set_home_position(self) -> bool:
        """현재 위치를 Home으로 설정"""
        try:
            # 현재 위치 가져오기
            async for position in self.drone.telemetry.position():
                lat = position.latitude_deg
                lon = position.longitude_deg
                alt = position.absolute_altitude_m
                break
            
            self.logger.info(f"Home 위치 설정: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            
            # Home 위치 설정 (ArduPilot 명령)
            # 실제로는 MAVLink 명령 전송 필요
            return True
            
        except Exception as e:
            self.logger.error(f"Home 설정 실패: {e}")
            return False
    
    async def return_to_launch(self) -> bool:
        """RTL (Return to Launch)"""
        try:
            self.logger.info("RTL 실행...")
            await self.drone.action.return_to_launch()
            return True
            
        except Exception as e:
            self.logger.error(f"RTL 실패: {e}")
            return False
    
    async def pause(self) -> bool:
        """현재 위치에서 정지 (호버링)"""
        try:
            await self.set_velocity_ned(0, 0, 0, 0)
            self.logger.info("정지 (호버링)")
            return True
            
        except Exception as e:
            self.logger.error(f"정지 실패: {e}")
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
                self.current_position = position
                self.current_altitude = position.relative_altitude_m
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
                self.current_heading = attitude.yaw_deg
                break
            
            # 배터리
            async for battery in self.drone.telemetry.battery():
                telemetry['battery'] = {
                    'voltage': battery.voltage_v,
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
            
            # 비행 모드
            async for mode in self.drone.telemetry.flight_mode():
                telemetry['flight_mode'] = str(mode)
                break
                
        except Exception as e:
            self.logger.error(f"텔레메트리 수집 오류: {e}")
        
        return telemetry
    
    async def stop_guided_mode(self) -> bool:
        """Guided 모드 종료"""
        try:
            if self.in_guided_mode:
                await self.drone.offboard.stop()
                self.in_guided_mode = False
                self.logger.info("GUIDED 모드 종료")
            return True
            
        except Exception as e:
            self.logger.error(f"GUIDED 모드 종료 실패: {e}")
            return False


# 사용 예제
async def main():
    """Guided 모드 테스트"""
    
    # 컨트롤러 생성
    guided = GuidedModeController("serial:///dev/ttyTHS1:115200")
    
    try:
        # FC 연결
        if not await guided.connect():
            print("FC 연결 실패!")
            return
        
        # Guided 모드 설정
        await guided.set_guided_mode()
        
        # 시동
        await guided.arm()
        await asyncio.sleep(2)
        
        # Guided 이륙 (5m)
        await guided.takeoff_guided(5.0)
        await asyncio.sleep(3)
        
        print("\n=== Guided 모드 제어 시작 ===\n")
        
        # 1. 속도 제어 테스트
        print("1. 전진 이동 (2m/s, 3초)")
        await guided.set_velocity_body(2.0, 0, 0, 0)
        await asyncio.sleep(3)
        
        print("2. 우측 이동 (1.5m/s, 3초)")
        await guided.set_velocity_body(0, 1.5, 0, 0)
        await asyncio.sleep(3)
        
        print("3. 제자리 회전 (90도)")
        await guided.rotate_yaw(90)
        await asyncio.sleep(2)
        
        # 2. 위치 제어 테스트
        print("4. 상대 위치 이동 (전방 5m)")
        await guided.move_relative(5.0, 0, 0)
        await asyncio.sleep(5)
        
        print("5. 상대 위치 이동 (우측 3m, 상승 2m)")
        await guided.move_relative(0, 3.0, -2.0)
        await asyncio.sleep(5)
        
        # 3. GPS 이동 테스트
        print("6. GPS 좌표 이동")
        await guided.goto_position_global(35.123456, 129.123456, 20.0)
        await asyncio.sleep(10)
        
        # 4. 텔레메트리 확인
        telemetry = await guided.get_telemetry()
        print(f"\n현재 상태:")
        print(f"  위치: {telemetry['position']}")
        print(f"  속도: {telemetry['velocity']}")
        print(f"  배터리: {telemetry['battery']}")
        print(f"  모드: {telemetry['flight_mode']}")
        
        # 5. RTL
        print("\n7. RTL (Return to Launch)")
        await guided.return_to_launch()
        await asyncio.sleep(15)
        
        # 착륙
        print("8. 착륙")
        await guided.land_guided()
        
        # 시동 끄기
        await guided.disarm()
        
        # Guided 모드 종료
        await guided.stop_guided_mode()
        
        print("\n=== 테스트 완료 ===")
        
    except KeyboardInterrupt:
        print("\n중단! 긴급 착륙...")
        await guided.pause()
        await guided.land_guided()
        await guided.disarm()
        
    except Exception as e:
        print(f"오류 발생: {e}")
        await guided.land_guided()
        await guided.disarm()


# 간단한 제어 예제
async def simple_control():
    """간단한 Guided 제어"""
    
    guided = GuidedModeController("serial:///dev/ttyTHS1:115200")
    
    # 연결 및 초기화
    await guided.connect()
    await guided.set_guided_mode()
    await guided.arm()
    
    # 이륙
    await guided.takeoff_guided(3.0)
    
    # 간단한 이동
    commands = [
        ("전진", 2.0, 0, 0),      # forward, right, down
        ("우측", 0, 1.5, 0),
        ("상승", 0, 0, -1.0),
        ("좌측", 0, -1.5, 0),
        ("하강", 0, 0, 1.0),
    ]
    
    for name, forward, right, down in commands:
        print(f"{name} 이동")
        await guided.set_velocity_body(forward, right, down)
        await asyncio.sleep(2)
    
    # 정지
    await guided.pause()
    
    # 착륙
    await guided.land_guided()
    await guided.disarm()


if __name__ == "__main__":
    # 메인 실행
    asyncio.run(main())
    
    # 또는 간단한 제어
