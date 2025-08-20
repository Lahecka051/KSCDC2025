# main_throttle_test.py

import asyncio
import logging

# 제공된 파일 중 FC와 통신하는 uart_communication 모듈을 사용합니다.
from uart_communication import FC_UART_Communication
from mavsdk.offboard import Attitude

# ##################################################################
# 🚨🚨🚨 경고 (WARNING) 🚨🚨🚨
# 이 스크립트는 모터에 직접적인 스로틀 명령을 인가합니다.
# 테스트 전, 드론의 프로펠러를 반드시 모두 제거하십시오.
# 프로펠러가 장착된 상태에서 실행하면 기체가 이륙하거나 뒤집어져
# 심각한 부상이나 재산 피해를 유발할 수 있습니다.
# ##################################################################


# --- 테스트 설정 (필요시 수정) ---
MIN_THROTTLE_PERCENT = 15  # 테스트 시작 스로틀 (%)
MAX_THROTTLE_PERCENT = 80  # 테스트 최대 스로틀 (%)
THROTTLE_STEP = 5          # 스로틀 증가/감소 단계 (%)
STEP_DURATION_S = 0.2      # 각 스로틀 단계를 유지하는 시간 (초)


# --- 로깅 기본 설정 ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] - %(message)s'
)


async def run_throttle_test():
    """
    드론에 연결하여 시동을 걸고, 스로틀을 15%에서 80%까지 올렸다가
    다시 15%로 내린 후 시동을 끄는 테스트를 수행합니다.
    """
    logger = logging.getLogger("ThrottleTest")
    
    logger.info("FC 통신 모듈을 초기화합니다...")
    fc_comm = FC_UART_Communication()

    logger.info("FC에 연결을 시도합니다...")
    if not await fc_comm.connect():
        logger.critical("FC 연결에 실패했습니다. 연결 상태와 포트 설정을 확인해주세요.")
        return

    # --- 최종 안전 확인 ---
    logger.warning("="*60)
    logger.warning("🚨🚨 스로틀 테스트를 시작합니다. 프로펠러가 모두 제거되었는지 마지막으로 확인하십시오. 🚨🚨")
    logger.warning("="*60)

    try:
        input("모든 안전 조치를 확인했으며, 테스트를 시작하려면 Enter 키를 누르세요...")
    except (KeyboardInterrupt, EOFError):
        logger.info("사용자에 의해 테스트가 취소되었습니다.")
        await fc_comm.disconnect()
        return

    # --- 스로틀 테스트 시작 ---
    try:
        # 1. 시동
        logger.info("시동(Arm)을 겁니다...")
        if not await fc_comm.arm():
            logger.error("시동에 실패했습니다. 테스트를 중단합니다.")
            await fc_comm.disconnect()
            return
        await asyncio.sleep(1)

        # 2. Offboard 모드 시작 (스로틀 직접 제어를 위함)
        logger.info("Offboard 모드를 시작합니다...")
        # 초기값으로 Attitude(자세) 제어, 스로틀 0% 설정
        await fc_comm.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
        await fc_comm.drone.offboard.start()
        logger.info("Offboard 모드 활성화.")
        await asyncio.sleep(1)

        # 3. 스로틀 올리기 (Ramp Up)
        logger.info(f"스로틀을 {MIN_THROTTLE_PERCENT}% 부터 {MAX_THROTTLE_PERCENT}% 까지 올립니다...")
        for throttle_percent in range(MIN_THROTTLE_PERCENT, MAX_THROTTLE_PERCENT + 1, THROTTLE_STEP):
            throttle_value = throttle_percent / 100.0  # 0.0 ~ 1.0 사이 값으로 변환
            print(f"  > 현재 스로틀: {throttle_percent}%")
            await fc_comm.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, throttle_value))
            await asyncio.sleep(STEP_DURATION_S)

        logger.info(f"최대 스로틀 {MAX_THROTTLE_PERCENT}% 도달.")
        await asyncio.sleep(1)

        # 4. 스로틀 내리기 (Ramp Down)
        logger.info(f"스로틀을 {MAX_THROTTLE_PERCENT}% 부터 {MIN_THROTTLE_PERCENT}% 까지 내립니다...")
        for throttle_percent in range(MAX_THROTTLE_PERCENT, MIN_THROTTLE_PERCENT - 1, -THROTTLE_STEP):
            throttle_value = throttle_percent / 100.0
            print(f"  > 현재 스로틀: {throttle_percent}%")
            await fc_comm.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, throttle_value))
            await asyncio.sleep(STEP_DURATION_S)
        
        logger.info(f"최저 스로틀 {MIN_THROTTLE_PERCENT}% 도달.")
        await asyncio.sleep(1)

        # 5. Offboard 모드 종료
        logger.info("Offboard 모드를 종료합니다...")
        await fc_comm.drone.offboard.stop()
        logger.info("Offboard 모드 비활성화.")
        await asyncio.sleep(1)

        # 6. 착륙 명령 (안전 절차)
        logger.info("안전 착륙 절차를 시작합니다...")
        await fc_comm.land()
        # 착륙이 감지될 때까지 잠시 대기
        await asyncio.sleep(3)

    except Exception as e:
        logger.error(f"테스트 실행 중 예기치 않은 오류가 발생했습니다: {e}")
        logger.info("안전을 위해 Offboard 모드를 중지하고 착륙을 시도합니다.")
        try:
            await fc_comm.drone.offboard.stop()
            await fc_comm.land()
        except Exception as safety_e:
            logger.error(f"비상 정지 중 오류 발생: {safety_e}")
            
    finally:
        # 7. 시동 끄기
        logger.info("시동(Disarm)을 끕니다...")
        await fc_comm.disarm()
        
        logger.info("FC와의 연결을 종료합니다.")
        await fc_comm.disconnect()
        
        logger.info("="*60)
        logger.info("스로틀 테스트가 모두 완료되었습니다.")
        logger.info("="*60)


if __name__ == "__main__":
    try:
        asyncio.run(run_throttle_test())
    except KeyboardInterrupt:
        logging.info("프로그램이 강제 종료되었습니다.")
