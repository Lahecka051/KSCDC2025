"""
motor_test.py
main_control.py를 활용한 모터 테스트 코드
모터 출력을 15%에서 90%까지 단계적으로 증가/감소
"""

import asyncio
import time
import logging
import signal
import sys
from typing import List
from main_control import Drone, ControlMode

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

class MotorTest:
    """모터 테스트 클래스"""
    
    def __init__(self):
        """초기화"""
        self.drone = Drone()
        self.logger = logging.getLogger(__name__)
        self.is_testing = False
        self.emergency_stop = False
        
        # 테스트 파라미터
        self.min_throttle = 15  # 최소 모터 출력 (%)
        self.max_throttle = 90  # 최대 모터 출력 (%)
        self.step_size = 5      # 증가/감소 단계 (%)
        self.step_duration = 3  # 각 단계 유지 시간 (초)
        
    async def initialize(self) -> bool:
        """시스템 초기화"""
        self.logger.info("="*60)
        self.logger.info("모터 테스트 시스템 초기화")
        self.logger.info("="*60)
        
        # 드론 초기화
        if not await self.drone.initialize():
            self.logger.error("드론 초기화 실패!")
            return False
        
        self.logger.info("초기화 완료!")
        return True
    
    async def basic_motor_test(self):
        """
        기본 모터 테스트
        15% → 90% → 15% 단계적 증가/감소
        """
        try:
            self.logger.info("\n" + "="*60)
            self.logger.info("기본 모터 테스트 시작")
            self.logger.info("="*60)
            
            # 시동
            if not await self.drone.arm_and_takeoff(2.0):
                self.logger.error("이륙 실패!")
                return
            
            self.is_testing = True
            
            # 초기 호버링
            self.logger.info("초기 호버링 (3초)")
            self.drone.cmd = ["level", "hover", 0, 30]
            await asyncio.sleep(3)
            
            # 상승 테스트 (15% → 90%)
            self.logger.info("\n>>> 모터 출력 상승 테스트 <<<")
            for throttle in range(self.min_throttle, self.max_throttle + 1, self.step_size):
                if self.emergency_stop:
                    break
                    
                self.logger.info(f"모터 출력: {throttle}%")
                self.drone.cmd = ["level", "hover", 0, throttle]
                await asyncio.sleep(self.step_duration)
            
            # 최대 출력 유지
            self.logger.info(f"\n최대 출력 유지: {self.max_throttle}% (5초)")
            await asyncio.sleep(5)
            
            # 하강 테스트 (90% → 15%)
            self.logger.info("\n>>> 모터 출력 하강 테스트 <<<")
            for throttle in range(self.max_throttle, self.min_throttle - 1, -self.step_size):
                if self.emergency_stop:
                    break
                    
                self.logger.info(f"모터 출력: {throttle}%")
                self.drone.cmd = ["level", "hover", 0, throttle]
                await asyncio.sleep(self.step_duration)
            
            # 테스트 완료
            self.logger.info("\n기본 모터 테스트 완료!")
            
        except Exception as e:
            self.logger.error(f"모터 테스트 오류: {e}")
        
        finally:
            self.is_testing = False
            await self.safe_landing()
    
    async def directional_motor_test(self):
        """
        방향별 모터 테스트
        각 방향으로 이동하며 모터 출력 변화
        """
        try:
            self.logger.info("\n" + "="*60)
            self.logger.info("방향별 모터 테스트 시작")
            self.logger.info("="*60)
            
            # 시동 및 이륙
            if not await self.drone.arm_and_takeoff(3.0):
                self.logger.error("이륙 실패!")
                return
            
            self.is_testing = True
            
            # 테스트 방향 및 출력
            directions = [
                ("전진", "forward"),
                ("후진", "backward"),
                ("좌측", "left"),
                ("우측", "right"),
                ("좌전방", "forward_left"),
                ("우전방", "forward_right"),
                ("좌후방", "backward_left"),
                ("우후방", "backward_right")
            ]
            
            for name, direction in directions:
                if self.emergency_stop:
                    break
                
                self.logger.info(f"\n>>> {name} 모터 테스트 <<<")
                
                # 출력 증가
                for throttle in range(20, 61, 10):
                    if self.emergency_stop:
                        break
                        
                    self.logger.info(f"{name}: {throttle}%")
                    self.drone.cmd = ["level", direction, 0, throttle]
                    await asyncio.sleep(2)
                
                # 호버링으로 복귀
                self.logger.info("호버링 복귀")
                self.drone.cmd = ["level", "hover", 0, 30]
                await asyncio.sleep(2)
            
            self.logger.info("\n방향별 모터 테스트 완료!")
            
        except Exception as e:
            self.logger.error(f"방향별 테스트 오류: {e}")
        
        finally:
            self.is_testing = False
            await self.safe_landing()
    
    async def vertical_motor_test(self):
        """
        수직 모터 테스트
        상승/하강 시 모터 출력 변화
        """
        try:
            self.logger.info("\n" + "="*60)
            self.logger.info("수직 모터 테스트 시작")
            self.logger.info("="*60)
            
            # 시동 및 이륙
            if not await self.drone.arm_and_takeoff(2.0):
                self.logger.error("이륙 실패!")
                return
            
            self.is_testing = True
            
            # 상승 테스트
            self.logger.info("\n>>> 상승 모터 테스트 <<<")
            for throttle in range(30, 71, 10):
                if self.emergency_stop:
                    break
                    
                self.logger.info(f"상승: {throttle}%")
                self.drone.cmd = ["up", "hover", 0, throttle]
                await asyncio.sleep(3)
            
            # 수평 유지
            self.logger.info("\n수평 유지 (5초)")
            self.drone.cmd = ["level", "hover", 0, 40]
            await asyncio.sleep(5)
            
            # 하강 테스트
            self.logger.info("\n>>> 하강 모터 테스트 <<<")
            for throttle in range(30, 61, 10):
                if self.emergency_stop:
                    break
                    
                self.logger.info(f"하강: {throttle}%")
                self.drone.cmd = ["down", "hover", 0, throttle]
                await asyncio.sleep(3)
            
            self.logger.info("\n수직 모터 테스트 완료!")
            
        except Exception as e:
            self.logger.error(f"수직 테스트 오류: {e}")
        
        finally:
            self.is_testing = False
            await self.safe_landing()
    
    async def cyclic_motor_test(self, cycles: int = 3):
        """
        반복 모터 테스트
        지정된 횟수만큼 상승/하강 반복
        
        Args:
            cycles: 반복 횟수
        """
        try:
            self.logger.info("\n" + "="*60)
            self.logger.info(f"반복 모터 테스트 시작 ({cycles}회)")
            self.logger.info("="*60)
            
            # 시동 및 이륙
            if not await self.drone.arm_and_takeoff(2.0):
                self.logger.error("이륙 실패!")
                return
            
            self.is_testing = True
            
            for cycle in range(1, cycles + 1):
                if self.emergency_stop:
                    break
                
                self.logger.info(f"\n=== 사이클 {cycle}/{cycles} ===")
                
                # 상승
                self.logger.info("출력 상승")
                for throttle in range(20, 71, 10):
                    if self.emergency_stop:
                        break
                    self.logger.info(f"  {throttle}%")
                    self.drone.cmd = ["level", "hover", 0, throttle]
                    await asyncio.sleep(2)
                
                # 하강
                self.logger.info("출력 하강")
                for throttle in range(70, 19, -10):
                    if self.emergency_stop:
                        break
                    self.logger.info(f"  {throttle}%")
                    self.drone.cmd = ["level", "hover", 0, throttle]
                    await asyncio.sleep(2)
            
            self.logger.info(f"\n반복 테스트 완료! (총 {cycles}회)")
            
        except Exception as e:
            self.logger.error(f"반복 테스트 오류: {e}")
        
        finally:
            self.is_testing = False
            await self.safe_landing()
    
    async def stress_test(self):
        """
        스트레스 테스트
        급격한 모터 출력 변화
        """
        try:
            self.logger.info("\n" + "="*60)
            self.logger.info("모터 스트레스 테스트 시작")
            self.logger.info("⚠️  주의: 급격한 출력 변화!")
            self.logger.info("="*60)
            
            # 확인
            await asyncio.sleep(3)
            
            # 시동 및 이륙
            if not await self.drone.arm_and_takeoff(3.0):
                self.logger.error("이륙 실패!")
                return
            
            self.is_testing = True
            
            # 스트레스 패턴
            patterns = [
                ("저속", 20),
                ("고속", 80),
                ("저속", 20),
                ("중속", 50),
                ("고속", 90),
                ("중속", 50),
                ("저속", 15),
                ("고속", 70),
                ("중속", 40),
            ]
            
            for name, throttle in patterns:
                if self.emergency_stop:
                    break
                    
                self.logger.info(f"{name}: {throttle}%")
                self.drone.cmd = ["level", "hover", 0, throttle]
                await asyncio.sleep(2)
            
            self.logger.info("\n스트레스 테스트 완료!")
            
        except Exception as e:
            self.logger.error(f"스트레스 테스트 오류: {e}")
        
        finally:
            self.is_testing = False
            await self.safe_landing()
    
    async def safe_landing(self):
        """안전 착륙"""
        try:
            self.logger.info("\n안전 착륙 시작...")
            
            # 호버링
            self.drone.cmd = ["level", "hover", 0, 30]
            await asyncio.sleep(2)
            
            # 천천히 출력 감소
            for throttle in range(30, 10, -5):
                self.drone.cmd = ["down", "hover", 0, throttle]
                await asyncio.sleep(1)
            
            # 착륙
            await self.drone.land_and_disarm()
            
        except Exception as e:
            self.logger.error(f"착륙 오류: {e}")
            await self.drone.emergency_stop()
    
    async def shutdown(self):
        """시스템 종료"""
        self.is_testing = False
        await self.drone.shutdown()
    
    def stop_test(self):
        """테스트 중단"""
        self.logger.warning("테스트 중단 요청!")
        self.emergency_stop = True


async def main():
    """메인 실행"""
    
    # 테스트 객체 생성
    tester = MotorTest()
    
    # 시그널 핸들러
    def signal_handler(sig, frame):
        print("\n\n⚠️  중단 신호 감지!")
        tester.stop_test()
        asyncio.create_task(tester.shutdown())
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # 초기화
    if not await tester.initialize():
        print("초기화 실패!")
        return
    
    # 테스트 메뉴
    print("\n" + "="*60)
    print("모터 테스트 프로그램")
    print("="*60)
    print("1. 기본 모터 테스트 (15% → 90% → 15%)")
    print("2. 방향별 모터 테스트")
    print("3. 수직 모터 테스트")
    print("4. 반복 모터 테스트")
    print("5. 스트레스 테스트")
    print("0. 종료")
    print("-"*60)
    
    choice = input("테스트 선택: ").strip()
    
    try:
        if choice == "1":
            await tester.basic_motor_test()
        elif choice == "2":
            await tester.directional_motor_test()
        elif choice == "3":
            await tester.vertical_motor_test()
        elif choice == "4":
            cycles = int(input("반복 횟수 (기본 3): ") or 3)
            await tester.cyclic_motor_test(cycles)
        elif choice == "5":
            confirm = input("⚠️  스트레스 테스트를 진행하시겠습니까? (y/n): ")
            if confirm.lower() == 'y':
                await tester.stress_test()
        else:
            print("종료합니다.")
    
    except Exception as e:
        print(f"오류: {e}")
        await tester.safe_landing()
    
    finally:
        await tester.shutdown()


# 간단한 테스트
async def quick_test():
    """빠른 모터 테스트"""
    
    tester = MotorTest()
    
    # 빠른 테스트 설정
    tester.min_throttle = 20
    tester.max_throttle = 60
    tester.step_size = 10
    tester.step_duration = 2
    
    await tester.initialize()
    await tester.basic_motor_test()
    await tester.shutdown()


# 커스텀 테스트
async def custom_test():
    """사용자 정의 모터 테스트"""
    
    drone = Drone()
    await drone.initialize()
    await drone.arm_and_takeoff(2.0)
    
    # 커스텀 패턴
    test_pattern = [
        ["level", "hover", 0, 15],     # 15%
        ["level", "hover", 0, 30],     # 30%
        ["level", "hover", 0, 45],     # 45%
        ["level", "hover", 0, 60],     # 60%
        ["level", "hover", 0, 75],     # 75%
        ["level", "hover", 0, 90],     # 90%
        ["level", "hover", 0, 75],     # 75%
        ["level", "hover", 0, 60],     # 60%
        ["level", "hover", 0, 45],     # 45%
        ["level", "hover", 0, 30],     # 30%
        ["level", "hover", 0, 15],     # 15%
    ]
    
    for cmd in test_pattern:
        print(f"모터 출력: {cmd[3]}%")
        drone.cmd = cmd
        await asyncio.sleep(3)
    
    await drone.land_and_disarm()
    await drone.shutdown()


if __name__ == "__main__":
    print("\n모터 테스트 모드 선택:")
    print("1. 전체 테스트 메뉴")
    print("2. 빠른 테스트")
    print("3. 커스텀 테스트")
    
    mode = input("선택: ").strip()
    
    if mode == "1":
        asyncio.run(main())
    elif mode == "2":
        asyncio.run(quick_test())
    elif mode == "3":
        asyncio.run(custom_test())
    else:
        print("종료")
