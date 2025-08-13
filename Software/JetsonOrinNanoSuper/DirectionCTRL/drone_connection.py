"""
drone_connection.py
드론과의 연결 및 통신을 관리하는 모듈
"""

import asyncio
import logging
from mavsdk import System
from mavsdk.telemetry import TelemetryError

# --- 로깅 설정 ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class DroneConnection:
    """드론 연결 및 통신을 관리하는 클래스"""
    
    def __init__(self, connection_string="serial:///dev/ttyTHS1:115200"):
        """
        DroneConnection 초기화
        
        Args:
            connection_string (str): 드론 연결 문자열
        """
        self.drone = System()
        self.connection_string = connection_string
        self.is_connected = False
        self.logger = logging.getLogger(__name__)
        
    async def connect(self, timeout=20.0):
        """
        드론에 연결하고 통신을 확인합니다.
        
        Args:
            timeout (float): 연결 타임아웃 시간 (초)
            
        Returns:
            bool: 연결 성공 여부
        """
        self.logger.info(f"FC에 연결을 시도합니다: {self.connection_string}")
        await self.drone.connect(system_address=self.connection_string)
        
        try:
            # 연결 상태 확인
            await asyncio.wait_for(self._wait_for_connection(), timeout=timeout)
            
            # 텔레메트리 수신으로 최종 연결 확인
            await self._verify_telemetry()
            
            self.is_connected = True
            self.logger.info("모든 연결 과정이 완료되었습니다.")
            return True
            
        except asyncio.TimeoutError:
            self.logger.error("연결 또는 텔레메트리 수신 시간 초과!")
            self.logger.error("FC를 재부팅하고 ArduPilot 설정을 다시 확인해 보세요.")
            return False
            
        except Exception as e:
            self.logger.error(f"예상치 못한 오류 발생: {e}")
            return False
    
    async def _wait_for_connection(self):
        """드론이 연결될 때까지 기다리는 내부 헬퍼 함수"""
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.logger.info("FC 발견 및 연결 성공!")
                return True
        return False
    
    async def _verify_telemetry(self):
        """텔레메트리 수신으로 양방향 통신 확인"""
        self.logger.info("FC로부터 텔레메트리(시동 상태) 수신을 시도합니다...")
        
        async for is_armed in self.drone.telemetry.armed():
            self.logger.info(f"텔레메트리 수신 성공! 현재 시동 상태: {'ARMED' if is_armed else 'DISARMED'}")
            break  # 첫 번째 수신 후 종료
    
    async def arm(self):
        """드론 시동 걸기"""
        if not self.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info("드론 시동을 걸고 있습니다...")
            await self.drone.action.arm()
            self.logger.info("드론 시동 완료!")
            return True
        except Exception as e:
            self.logger.error(f"시동 실패: {e}")
            return False
    
    async def disarm(self):
        """드론 시동 끄기"""
        if not self.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info("드론 시동을 끄고 있습니다...")
            await self.drone.action.disarm()
            self.logger.info("드론 시동 꺼짐!")
            return True
        except Exception as e:
            self.logger.error(f"시동 끄기 실패: {e}")
            return False
    
    async def takeoff(self, altitude=2.0):
        """
        드론 이륙
        
        Args:
            altitude (float): 이륙 고도 (미터)
        """
        if not self.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"{altitude}m 고도로 이륙합니다...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            self.logger.info("이륙 완료!")
            return True
        except Exception as e:
            self.logger.error(f"이륙 실패: {e}")
            return False
    
    async def land(self):
        """드론 착륙"""
        if not self.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info("착륙을 시작합니다...")
            await self.drone.action.land()
            self.logger.info("착륙 완료!")
            return True
        except Exception as e:
            self.logger.error(f"착륙 실패: {e}")
            return False
    
    async def get_telemetry_data(self):
        """
        현재 텔레메트리 데이터 가져오기
        
        Returns:
            dict: 텔레메트리 데이터
        """
        if not self.is_connected:
            return None
            
        telemetry_data = {}
        
        try:
            # 위치 정보
            async for position in self.drone.telemetry.position():
                telemetry_data['position'] = {
                    'latitude': position.latitude_deg,
                    'longitude': position.longitude_deg,
                    'altitude': position.relative_altitude_m
                }
                break
            
            # 자세 정보
            async for attitude in self.drone.telemetry.attitude_euler():
                telemetry_data['attitude'] = {
                    'roll': attitude.roll_deg,
                    'pitch': attitude.pitch_deg,
                    'yaw': attitude.yaw_deg
                }
                break
            
            # 배터리 정보
            async for battery in self.drone.telemetry.battery():
                telemetry_data['battery'] = {
                    'voltage': battery.voltage_v,
                    'remaining_percent': battery.remaining_percent
                }
                break
                
            # GPS 정보
            async for gps_info in self.drone.telemetry.gps_info():
                telemetry_data['gps'] = {
                    'num_satellites': gps_info.num_satellites,
                    'fix_type': gps_info.fix_type
                }
                break
                
        except Exception as e:
            self.logger.error(f"텔레메트리 데이터 수집 실패: {e}")
            
        return telemetry_data
    
    def get_drone_instance(self):
        """
        MAVSDK 드론 인스턴스 반환
        
        Returns:
            System: MAVSDK 드론 인스턴스
        """
        return self.drone
