"""
simple_up_down.py
간단한 상승/하강 테스트 코드
시동 → 2m 상승 → 2m 하강 → 시동 끄기
"""

import asyncio
import logging
from drone_connection import DroneConnection
from drone_control import DroneController

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

async def up_down_test():
    """
    간단한 상승/하강 테스트
    """
    
    # 드론 연결
    connection = DroneConnection("serial:///dev/ttyTHS0:115200")
    
    try:
        print("=" * 40)
        print("  간단한 상승/하강 테스트")
        print("=" * 40)
        
        # 1. 드론 연결
        print("\n[1/5] 드론 연결 중...")
        if not await connection.connect(timeout=20.0):
            print("❌ 드론 연결 실패!")
            return
        print("✅ 연결 성공")
        
        # 컨트롤러 생성
        controller = DroneController(connection)
        
        # 2. 시동 걸기
        print("\n[2/5] 시동 걸기...")
        if not await connection.arm():
            print("❌ 시동 실패!")
            return
        print("✅ 시동 완료")
        await asyncio.sleep(2)  # 안정화 대기
        
        # 3. 2m 상승
        print("\n[3/5] 2m 상승 중...")
        print("  속도: 0.5 m/s")
        
        # Offboard 모드 시작 (정밀 제어)
        await controller.start_offboard_mode()
        await asyncio.sleep(1)
        
        # 상승 실행
        await controller.move_up(distance_m=2.0, speed_m_s=0.5)
        print("✅ 2m 상승 완료")
        
        # 잠시 호버링
        print("\n  3초간 호버링...")
        await asyncio.sleep(3)
        
        # 4. 2m 하강
        print("\n[4/5] 2m 하강 중...")
        print("  속도: 0.3 m/s (안전을 위해 천천히)")
        
        await controller.move_down(distance_m=2.0, speed_m_s=0.3)
        print("✅ 착지 완료")
        
        # Offboard 모드 종료
        await controller.stop_offboard_mode()
        await asyncio.sleep(2)
        
        # 5. 시동 끄기
        print("\n[5/5] 시동 끄기...")
        if not await connection.disarm():
            print("⚠️  시동 끄기 실패 - 수동으로 끄세요")
        else:
            print("✅ 시동 꺼짐")
        
        print("\n" + "=" * 40)
        print("  테스트 완료!")
        print("=" * 40)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  중단됨! 긴급 하강...")
        try:
            await controller.stop()
            await controller.move_down(distance_m=3.0, speed_m_s=0.5)
            await connection.disarm()
        except:
            print("❌ 수동 조작 필요!")
    
    except Exception as e:
        print(f"\n❌ 오류: {e}")
        try:
            await controller.stop()
            await connection.disarm()
        except:
            pass


async def main():
    """메인 실행 함수"""
    
    print("\n" + "=" * 40)
    print("  드론 상승/하강 테스트 프로그램")
    print("=" * 40)
    print("\n⚠️  안전 확인:")
    print("  • 위쪽 공간 최소 3m 확보")
    print("  • 주변 장애물 제거")
    print("  • 배터리 충분한지 확인")
    
    response = input("\n준비되었습니까? (y/n): ")
    
    if response.lower() == 'y':
        await up_down_test()
    else:
        print("테스트를 취소했습니다.")


if __name__ == "__main__":
    asyncio.run(main())
