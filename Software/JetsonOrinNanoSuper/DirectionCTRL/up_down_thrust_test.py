"""
up_down_thrust_test.py

main_control.py의 Drone 클래스를 모듈로 불러와
모터 부하(추력)를 이용한 간단한 상승/하강을 테스트합니다.

실행 전 확인 사항:
- 이 파일과 동일한 위치에 'main_control.py', 'drone_control.py', 
  'drone_communication.py' 파일이 있어야 합니다.
- 필요한 라이브러리가 설치되어 있어야 합니다. (mavsdk, pyserial 등)
"""
import asyncio
import logging
import signal
import sys

# --- 로깅 설정 ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(message)s'
)

# --- main_control 모듈에서 Drone 클래스 가져오기 ---
try:
    from main_control import Drone
except ImportError:
    logging.error("="*60)
    logging.error("오류: main_control.py 파일을 찾을 수 없습니다.")
    logging.error("이 스크립트와 같은 폴더에 필수 파일들이 있는지 확인하세요.")
    logging.error("필수 파일: main_control.py, drone_control.py, drone_communication.py")
    logging.error("="*60)
    sys.exit(1)


async def run_thrust_test():
    """
    모터 부하(추력)를 조절하여 상승 및 하강을 테스트하는 메인 함수
    """
    drone = Drone()

    # --- 비상 종료를 위한 시그널 핸들러 설정 ---
    def handle_signal(sig, frame):
        print("\n[비상] 종료 신호 감지! 즉시 착륙 및 시스템 종료를 시도합니다.")
        # 비동기 함수를 안전하게 호출하기 위해 루프에서 실행
        if drone.is_running:
            asyncio.create_task(drone.land_and_disarm())
            asyncio.create_task(drone.shutdown())
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)

    try:
        # --- 1. 시스템 초기화 ---
        # Drone 클래스 내에서 FC 연결 및 UART 통신이 모두 초기화됩니다.
        if not await drone.initialize():
            logging.error("드론 시스템 초기화에 실패했습니다.")
            return

        # --- 2. 시동 및 이륙 ---
        # arm_and_takeoff는 시동, 이륙, Offboard 모드 시작까지 포함합니다.
        print("\n[단계 1/4] 시동 및 이륙을 시작합니다...")
        if not await drone.arm_and_takeoff(altitude=2.0):
            logging.error("시동 및 이륙에 실패했습니다.")
            await drone.shutdown()
            return
        print("✅ 이륙 완료 및 Offboard 모드 활성화")

        # --- 3. 추력 제어 비행 시퀀스 ---
        # 상승 (호버링 추력 50%보다 높은 65% 설정)
        print("\n[단계 2/4] 상승 시작 (추력 65%로 4초간)")
        drone.cmd = ["up", "hover", 0, 65]
        await asyncio.sleep(4)

        # 호버링 (안정화를 위해 50% 추력으로 잠시 대기)
        print("\n[단계 3/4] 호버링 (추력 50%로 3초간)")
        drone.cmd = ["level", "hover", 0, 50]
        await asyncio.sleep(3)

        # 하강 (호버링 추력 50%보다 낮은 40% 설정)
        print("\n[단계 4/4] 하강 시작 (추력 40%로 4초간)")
        drone.cmd = ["down", "hover", 0, 40]
        await asyncio.sleep(4)

        print("\n✅ 자동 비행 시퀀스가 완료되었습니다.")

    except Exception as e:
        logging.error(f"테스트 중 예상치 못한 오류 발생: {e}")
        # 오류 발생 시 안전하게 착륙 시도
        if drone.is_flying:
            await drone.land_and_disarm()
    finally:
        # --- 4. 시스템 종료 ---
        logging.info("테스트 완료. 시스템을 안전하게 종료합니다.")
        if drone.is_flying:
            await drone.land_and_disarm()
        await drone.shutdown()


if __name__ == "__main__":
    print("\n" + "=" * 50)
    print("  드론 모터 부하(추력) 상승/하강 테스트")
    print("=" * 50)
    print("\n⚠️  안전 경고:")
    print("  - 프로펠러로부터 안전 거리를 유지하세요.")
    print("  - 테스트를 위한 충분한 공간이 확보되었는지 확인하세요.")
    print("  - 비상 상황 시 'Ctrl + C'를 눌러 즉시 종료할 수 있습니다.")
    
    response = input("\n테스트를 시작하시겠습니까? (y/n): ").strip().lower()
    
    if response == 'y':
        asyncio.run(run_thrust_test())
    else:
        print("테스트를 취소했습니다.")
