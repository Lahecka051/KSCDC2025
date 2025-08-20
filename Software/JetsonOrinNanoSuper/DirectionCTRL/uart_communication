"""
uart_communication.py
젯슨과 비행 컨트롤러(FC) 간의 UART(시리얼) 통신을 담당하는 모듈.
MAVSDK 라이브러리를 사용하여 MAVLink 프로토콜로 드론을 제어하고 데이터를 수신합니다.
"""

import asyncio
import logging
from dataclasses import dataclass
from typing import Optional

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.telemetry import FlightMode

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

@dataclass
class FlightStatus:
    """드론의 주요 상태 정보를 담는 데이터 클래스"""
    is_armed: bool = False
    is_flying: bool = False
    flight_mode: str = "UNKNOWN"
    latitude: float = 0.0
    longitude: float = 0.0
    relative_altitude_m: float = 0.0
    battery_percent: float = 0.0
    gps_satellites: int = 0

class FC_UART_Communication:
    """
    비행 컨트롤러(FC)와의 UART 통신을 관리하는 클래스.
    """
    
    def __init__(self, serial_port: str = "/dev/ttyTHS1", baud_rate: int = 115200):
        """
        초기화
        
        Args:
            serial_port (str): FC와 연결된 시리얼 포트 경로
            baud_rate (int): 통신 속도
        """
        self.connection_string = f"serial://{serial_port}:{baud_rate}"
        self.drone = System()
        self.is_connected = False
        self.logger = logging.getLogger(self.__class__.__name__)

    async def connect(self) -> bool:
        """비행 컨트롤러에 연결을 시도합니다."""
        self.logger.info(f"FC에 연결을 시도합니다: {self.connection_string}")
        try:
            await self.drone.connect(system_address=self.connection_string)

            # 연결 상태를 비동기적으로 확인
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("FC 연결 성공!")
                    self.is_connected = True
                    # 초기 텔레메트리 수신 대기
                    await asyncio.sleep(1)
                    return True
            
        except Exception as e:
            self.logger.error(f"FC 연결 실패: {e}")
            return False
        
        return False

    async def disconnect(self):
        """FC와의 연결을 해제합니다."""
        self.logger.info("FC와의 연결을 해제합니다.")
        # MAVSDK는 별도의 disconnect 함수가 없으며, 프로그램 종료 시 자동으로 처리됩니다.
        self.is_connected = False

    async def get_status(self) -> FlightStatus:
        """드론의 현재 상태를 종합하여 반환합니다."""
        if not self.is_connected:
            return FlightStatus()

        try:
            # 여러 텔레메트리 정보를 동시에 요청하여 효율적으로 가져옵니다.
            tasks = {
                "armed": asyncio.create_task(self.drone.telemetry.armed().__anext__()),
                "in_air": asyncio.create_task(self.drone.telemetry.in_air().__anext__()),
                "flight_mode": asyncio.create_task(self.drone.telemetry.flight_mode().__anext__()),
                "position": asyncio.create_task(self.drone.telemetry.position().__anext__()),
                "battery": asyncio.create_task(self.drone.telemetry.battery().__anext__()),
                "gps_info": asyncio.create_task(self.drone.telemetry.gps_info().__anext__()),
            }
            
            results = await asyncio.gather(*tasks.values())
            
            armed, in_air, flight_mode, position, battery, gps_info = results

            return FlightStatus(
                is_armed=armed,
                is_flying=in_air,
                flight_mode=str(flight_mode),
                latitude=position.latitude_deg,
                longitude=position.longitude_deg,
                relative_altitude_m=position.relative_altitude_m,
                battery_percent=battery.remaining_percent,
                gps_satellites=gps_info.num_satellites
            )
        except Exception as e:
            self.logger.error(f"상태 정보 수신 실패: {e}")
            return FlightStatus()

    async def arm(self) -> bool:
        """드론의 시동을 겁니다 (Arm)."""
        if not self.is_connected:
            self.logger.error("연결되지 않은 상태에서는 시동을 걸 수 없습니다.")
            return False
        try:
            self.logger.info("시동(Arm)을 시도합니다...")
            await self.drone.action.arm()
            self.logger.info("시동 성공.")
            return True
        except ActionError as e:
            self.logger.error(f"시동 실패: {e}")
            return False

    async def disarm(self) -> bool:
        """드론의 시동을 끕니다 (Disarm)."""
        if not self.is_connected:
            self.logger.error("연결되지 않은 상태에서는 시동을 끌 수 없습니다.")
            return False
        try:
            self.logger.info("시동 끄기(Disarm)를 시도합니다...")
            await self.drone.action.disarm()
            self.logger.info("시동 끄기 성공.")
            return True
        except ActionError as e:
            self.logger.error(f"시동 끄기 실패: {e}")
            return False

    async def takeoff(self, altitude_m: float = 5.0) -> bool:
        """지정된 고도로 이륙합니다."""
        if not self.is_connected:
            self.logger.error("연결되지 않은 상태에서는 이륙할 수 없습니다.")
            return False
        try:
            self.logger.info(f"{altitude_m}m 고도로 이륙을 시도합니다...")
            await self.drone.action.set_takeoff_altitude(altitude_m)
            await self.drone.action.takeoff()
            self.logger.info("이륙 명령 전송 완료.")
            return True
        except ActionError as e:
            self.logger.error(f"이륙 실패: {e}")
            return False

    async def land(self) -> bool:
        """현재 위치에 착륙합니다."""
        if not self.is_connected:
            self.logger.error("연결되지 않은 상태에서는 착륙할 수 없습니다.")
            return False
        try:
            self.logger.info("착륙을 시도합니다...")
            await self.drone.action.land()
            self.logger.info("착륙 명령 전송 완료.")
            return True
        except ActionError as e:
            self.logger.error(f"착륙 실패: {e}")
            return False

    async def goto_location(self, latitude_deg: float, longitude_deg: float, altitude_m: float, yaw_deg: float = 0.0):
        """지정된 GPS 좌표와 고도로 이동합니다."""
        if not self.is_connected:
            self.logger.error("연결되지 않은 상태에서는 이동할 수 없습니다.")
            return False
        try:
            self.logger.info(f"좌표 이동: ({latitude_deg}, {longitude_deg}) @ {altitude_m}m")
            await self.drone.action.goto_location(latitude_deg, longitude_deg, altitude_m, yaw_deg)
            self.logger.info("좌표 이동 명령 전송 완료.")
            return True
        except ActionError as e:
            self.logger.error(f"좌표 이동 실패: {e}")
            return False
