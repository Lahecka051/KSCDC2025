"""
main_control.py
H743v2 FC 드론의 메인 컨트롤 프로그램
drone.cmd = [4가지 데이터] 형식으로 간단한 제어
"""

import asyncio
import threading
import queue
import time
import logging
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import signal
import sys

# 모듈 임포트
from communication import UARTCommunication
from drone_control import DroneConnection, DroneController

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

class ControlMode(Enum):
    """제어 모드"""
    MANUAL = "manual"
    AUTO = "auto"
    EMERGENCY = "emergency"

class Drone:
    """간소화된 드론 인터페이스"""
    
    def __init__(self):
        """초기화"""
        self.logger = logging.getLogger(__name__)
        
        # 드론 연결
        self.connection = DroneConnection("serial:///dev/ttyTHS1:115200")
        self.controller = None
        
        # UART 통신
        self.uart = UARTCommunication("/dev/ttyTHS1", 115200)
        
        # 현재 명령 (외부에서 설정)
        self._cmd = ["level", "hover", 0, 0]
        
        # 제어 모드
        self.mode = ControlMode.MANUAL
        
        # 상태
        self.is_running = False
        self.is_armed = False
        self.is_flying = False
        
        # 태스크
        self.control_task = None
        self.gps_task = None
        
        # 통계
        self.stats = {
            'commands_executed': 0,
            'gps_updates': 0,
            'errors': 0
        }
    
    @property
    def cmd(self):
        """현재 명령 반환"""
        return self._cmd
    
    @cmd.setter
    def cmd(self, value: List):
        """
        명령 설정 - 외부에서 직접 설정
        
        Args:
            value: 
                - 일반 제어: [vertical, horizontal, rotation, motor_percent]
                - GPS 이동: ["gps", latitude, longitude] 또는 ["gps", latitude, longitude, altitude]
        
        Examples:
            drone.cmd = ["up", "forward_left", 30, 50]  # 일반 제어
            drone.cmd = ["gps", 35.123456, 129.123456]  # GPS 이동 (현재 고도)
            drone.cmd = ["gps", 35.123456, 129.123456, 10.0]  # GPS 이동 (지정 고도)
        """
        if len(value) < 3 or len(value) > 4:
            self.logger.error(f"잘못된 명령 형식: {value}")
            return
        
        # GPS 명령 검증
        if value[0] == "gps":
            if len(value) < 3:
                self.logger.error("GPS 명령은 최소 3개 값 필요: ['gps', lat, lon]")
                return
            try:
                lat = float(value[1])
                lon = float(value[2])
                if len(value) == 4:
                    alt = float(value[3])
            except (ValueError, TypeError):
                self.logger.error("GPS 좌표는 숫자여야 합니다")
                return
        
        # 일반 명령은 4개 값 필요
        elif len(value) != 4:
            self.logger.error(f"일반 명령은 4개 값 필요: {value}")
            return
        
        self._cmd = value
        self.logger.info(f"명령 설정: {value}")
    
    async def initialize(self) -> bool:
        """시스템 초기화"""
        self.logger.info("="*50)
        self.logger.info("드론 시스템 초기화")
        self.logger.info("="*50)
        
        # FC 연결
        self.logger.info("1. FC 연결...")
        if not await self.connection.connect():
            self.logger.error("FC 연결 실패!")
            return False
        
        # 컨트롤러 생성
        self.controller = DroneController(self.connection)
        
        # UART 시작
        self.logger.info("2. UART 통신...")
        if not self.uart.start():
            self.logger.error("UART 시작 실패!")
            return False
        
        self.is_running = True
        
        # 태스크 시작
        self.control_task = asyncio.create_task(self._control_loop())
        self.gps_task = asyncio.create_task(self._gps_loop())
        
        self.logger.info("3. 초기화 완료!")
        self.logger.info("="*50)
        return True
    
    async def _control_loop(self):
        """제어 루프 - cmd 실행"""
        while self.is_running:
            try:
                if self.is_flying and self.mode != ControlMode.EMERGENCY:
                    # 현재 cmd 실행
                    success = await self.controller.execute_cmd(self._cmd)
                    if success:
                        self.stats['commands_executed'] += 1
                    else:
                        self.stats['errors'] += 1
                
                await asyncio.sleep(0.05)  # 20Hz
                
            except Exception as e:
                self.logger.error(f"제어 루프 오류: {e}")
                self.stats['errors'] += 1
                await asyncio.sleep(0.1)
    
    async def _gps_loop(self):
        """GPS 데이터 전송"""
        while self.is_running:
            try:
                telemetry = await self.connection.get_telemetry()
                
                if telemetry:
                    self.uart.send_gps_data(telemetry)
                    self.stats['gps_updates'] += 1
                
                await asyncio.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.logger.error(f"GPS 오류: {e}")
                await asyncio.sleep(1)
    
    async def arm_and_takeoff(self, altitude=2.0) -> bool:
        """시동 및 이륙"""
        try:
            self.logger.info("시동...")
            if not await self.connection.arm():
                return False
            
            self.is_armed = True
            await asyncio.sleep(2)
            
            self.logger.info(f"이륙... (고도: {altitude}m)")
            if not await self.connection.takeoff(altitude):
                await self.connection.disarm()
                return False
            
            await asyncio.sleep(5)
            
            # Offboard 모드
            await self.controller.start_offboard()
            
            self.is_flying = True
            self.logger.info("이륙 완료!")
            
            # 초기 호버링
            self.cmd = ["level", "hover", 0, 0]
            
            return True
            
        except Exception as e:
            self.logger.error(f"이륙 실패: {e}")
            return False
    
    async def land_and_disarm(self) -> bool:
        """착륙 및 시동 끄기"""
        try:
            self.logger.info("착륙...")
            
            # 호버링
            self.cmd = ["level", "hover", 0, 0]
            await asyncio.sleep(1)
            
            # Offboard 종료
            await self.controller.stop_offboard()
            
            # 착륙
            await self.connection.land()
            self.is_flying = False
            await asyncio.sleep(5)
            
            # 시동 끄기
            await self.connection.disarm()
            self.is_armed = False
            
            self.logger.info("착륙 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"착륙 실패: {e}")
            return False
    
    async def emergency_stop(self):
        """긴급 정지"""
        self.logger.warning("!!! 긴급 정지 !!!")
        self.mode = ControlMode.EMERGENCY
        self.cmd = ["level", "hover", 0, 0]
        await self.controller.emergency_stop()
    
    async def shutdown(self):
        """시스템 종료"""
        self.logger.info("시스템 종료...")
        
        self.is_running = False
        
        if self.control_task:
            self.control_task.cancel()
        if self.gps_task:
            self.gps_task.cancel()
        
        self.uart.stop()
        
        self.logger.info(f"통계: {self.stats}")
        self.logger.info("종료 완료")
    
    def get_status(self) -> Dict[str, Any]:
        """상태 반환"""
        return {
            'mode': self.mode.value,
            'armed': self.is_armed,
            'flying': self.is_flying,
            'current_cmd': self._cmd,
            'stats': self.stats
        }


# 메인 실행
async def main():
    """메인 실행 예제"""
    
    # 드론 객체 생성
    drone = Drone()
    
    # 종료 시그널 핸들러
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
        # 시동 및 이륙
        await drone.arm_and_takeoff(altitude=2.0)
        
        print("\n=== 드론 제어 시작 ===")
        print("drone.cmd = [vertical, horizontal, rotation, motor_percent]")
        print("예: drone.cmd = ['up', 'forward_left', 30, 50]\n")
        
        # 예제 1: 간단한 명령
        print("전진 50%")
        drone.cmd = ["level", "forward", 0, 50]
        await asyncio.sleep(3)
        
        print("우상향 대각선 60%")
        drone.cmd = ["up", "forward_right", 0, 60]
        await asyncio.sleep(3)
        
        print("제자리 90도 회전")
        drone.cmd = ["level", "hover", 90, 0]
        await asyncio.sleep(2)
        
        print("좌측 이동 40%")
        drone.cmd = ["level", "left", 0, 40]
        await asyncio.sleep(3)
        
        print("하강하며 후진 30%")
        drone.cmd = ["down", "backward", 0, 30]
        await asyncio.sleep(3)
        
        # 예제 2: GPS 이동
        print("GPS 좌표로 이동")
        drone.cmd = ["gps", 35.123456, 129.123456]  # 현재 고도 유지
        await asyncio.sleep(10)
        
        print("다른 GPS 좌표로 이동 (고도 지정)")
        drone.cmd = ["gps", 35.123556, 129.123556, 15.0]  # 15m 고도
        await asyncio.sleep(10)
        
        print("정지")
        drone.cmd = ["level", "hover", 0, 0]
        await asyncio.sleep(2)
        
        # 착륙
        await drone.land_and_disarm()
        
    except Exception as e:
        print(f"오류: {e}")
        await drone.emergency_stop()
        await drone.land_and_disarm()
    
    finally:
        await drone.shutdown()


# 외부 모듈에서 사용하는 예제
def external_module_example():
    """다른 Python 모듈에서 제어하는 예제"""
    
    async def control_drone():
        # 드론 객체 생성
        drone = Drone()
        
        # 초기화
        if not await drone.initialize():
            return
        
        # 이륙
        await drone.arm_and_takeoff(2.0)
        
        # 1. Thinker GPS 모듈에서 받은 좌표
        gps_waypoints = [
            [35.123456, 129.123456],  # 첫 번째 웨이포인트
            [35.123556, 129.123556],  # 두 번째 웨이포인트
            [35.123656, 129.123656],  # 세 번째 웨이포인트
        ]
        
        for waypoint in gps_waypoints:
            print(f"이동: {waypoint}")
            drone.cmd = ["gps", waypoint[0], waypoint[1]]
            await asyncio.sleep(15)  # 이동 대기
        
        # 2. 고도를 포함한 GPS 이동
        drone.cmd = ["gps", 35.123756, 129.123756, 20.0]  # 20m 고도
        await asyncio.sleep(15)
        
        # 3. 일반 제어와 GPS 혼합
        commands = [
            ["level", "forward", 0, 60],  # 일반 제어
            ["gps", 35.123856, 129.123856],  # GPS 이동
            ["up", "hover", 90, 0],  # 회전 상승
            ["gps", 35.123456, 129.123456, 5.0],  # 복귀
        ]
        
        for cmd in commands:
            drone.cmd = cmd
            await asyncio.sleep(10)
        
        # 착륙
        await drone.land_and_disarm()
        await drone.shutdown()
    
    # 실행
    asyncio.run(control_drone())


# GPS 자율비행 통합 예제
class GPSAutonomousFlight:
    """GPS 기반 자율비행 클래스"""
    
    def __init__(self):
        self.drone = Drone()
        self.mission_waypoints = []
        self.current_waypoint_idx = 0
        
    def load_mission(self, waypoints):
        """
        미션 웨이포인트 로드
        
        Args:
            waypoints: [[lat, lon, alt], ...] 형식의 리스트
        """
        self.mission_waypoints = waypoints
        self.current_waypoint_idx = 0
        print(f"미션 로드: {len(waypoints)}개 웨이포인트")
    
    async def execute_mission(self):
        """미션 실행"""
        # 초기화
        if not await self.drone.initialize():
            print("초기화 실패")
            return
        
        # 이륙
        await self.drone.arm_and_takeoff(5.0)
        
        try:
            # 각 웨이포인트로 이동
            for idx, waypoint in enumerate(self.mission_waypoints):
                print(f"\n웨이포인트 {idx+1}/{len(self.mission_waypoints)}")
                
                if len(waypoint) == 2:
                    # 위도, 경도만
                    self.drone.cmd = ["gps", waypoint[0], waypoint[1]]
                elif len(waypoint) == 3:
                    # 위도, 경도, 고도
                    self.drone.cmd = ["gps", waypoint[0], waypoint[1], waypoint[2]]
                
                # 도착 대기 (실제로는 GPS 거리 확인 필요)
                await asyncio.sleep(20)
                
                # 도착 확인
                print(f"웨이포인트 {idx+1} 도착")
            
            print("\n미션 완료!")
            
        except Exception as e:
            print(f"미션 실패: {e}")
            await self.drone.emergency_stop()
        
        finally:
            # 착륙
            await self.drone.land_and_disarm()
            await self.drone.shutdown()
    
    async def patrol_mission(self):
        """순찰 미션 (반복)"""
        patrol_points = [
            [35.123456, 129.123456, 10.0],
            [35.123556, 129.123456, 10.0],
            [35.123556, 129.123556, 10.0],
            [35.123456, 129.123556, 10.0],
        ]
        
        self.load_mission(patrol_points)
        
        # 초기화
        if not await self.drone.initialize():
            return
        
        await self.drone.arm_and_takeoff(5.0)
        
        try:
            # 3회 순찰
            for patrol in range(3):
                print(f"\n=== 순찰 {patrol+1}/3 ===")
                
                for idx, point in enumerate(patrol_points):
                    print(f"순찰점 {idx+1}")
                    self.drone.cmd = ["gps", point[0], point[1], point[2]]
                    await asyncio.sleep(15)
                
        finally:
            await self.drone.land_and_disarm()
            await self.drone.shutdown()


# Thinker 모듈과 통합 예제
async def thinker_integration():
    """Thinker GPS 모듈과 통합"""
    
    drone = Drone()
    await drone.initialize()
    await drone.arm_and_takeoff(5.0)
    
    # Thinker에서 받은 GPS 경로
    def get_thinker_path():
        # 실제로는 Thinker 모듈에서 계산된 경로
        return [
            {"lat": 35.123456, "lon": 129.123456, "alt": 10.0},
            {"lat": 35.123556, "lon": 129.123556, "alt": 15.0},
            {"lat": 35.123656, "lon": 129.123656, "alt": 10.0},
        ]
    
    # Thinker 경로 실행
    thinker_path = get_thinker_path()
    
    for point in thinker_path:
        print(f"Thinker 경로점: {point}")
        drone.cmd = ["gps", point["lat"], point["lon"], point["alt"]]
        
        # 도착 확인 (간단한 대기)
        await asyncio.sleep(15)
        
        # 또는 거리 기반 확인
        while True:
            telemetry = await drone.connection.get_telemetry()
            current_pos = telemetry.get('position', {})
            
            # 거리 계산 (간단한 예)
            distance = calculate_distance(
                current_pos.get('lat'), current_pos.get('lon'),
                point["lat"], point["lon"]
            )
            
            if distance < 2.0:  # 2m 이내 도착
                break
            
            await asyncio.sleep(1)
    
    await drone.land_and_disarm()
    await drone.shutdown()


def calculate_distance(lat1, lon1, lat2, lon2):
    """두 GPS 좌표 간 거리 계산 (미터)"""
    from math import radians, sin, cos, sqrt, atan2
    
    R = 6371000  # 지구 반경 (미터)
    
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    return R * c


# 아주 간단한 사용법
async def simplest_usage():
    """가장 간단한 사용법"""
    
    drone = Drone()
    await drone.initialize()
    await drone.arm_and_takeoff()
    
    # 4가지 데이터로 제어
    drone.cmd = ["up", "forward_left", 30, 50]
    await asyncio.sleep(3)
    
    drone.cmd = ["level", "hover", 0, 0]
    await asyncio.sleep(1)
    
    await drone.land_and_disarm()
    await drone.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
