
import asyncio
import logging
from mavsdk import System
from mavsdk.telemetry import TelemetryError
# --- 로깅 설정 ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
async def wait_for_connection(drone):
    """드론이 연결될 때까지 기다리는 헬퍼 함수"""
    async for state in drone.core.connection_state():
        if state.is_connected:
            logging.info("FC 발견 및 연결 성공!")
            return True
    return False
async def run():
    """
    드론에 연결하고 텔레메트리 수신을 최종 확인하는 함수
    """
    drone = System()
    connection_string = "serial:///dev/ttyTHS0:115200"

    logging.info(f"FC에 연결을 시도합니다: {connection_string}")
    await drone.connect(system_address=connection_string)
    try:
        # --- 1. 연결 상태 확인 (20초 타임아웃) ---
        await asyncio.wait_for(wait_for_connection(drone), timeout=20.0)

        # --- 2. 텔레메트리 수신으로 최종 연결 확인 (수정된 부분) ---
        logging.info("FC로부터 텔레메트리(시동 상태) 수신을 시도합니다...")
        # 시동 상태(armed) 정보를 한 번 받아오는 것으로 양방향 통신을 최종 확인합니다.
        async for is_armed in drone.telemetry.armed():
            logging.info(f"텔레메트리 수신 성공! 현재 시동 상태: {'ARMED' if is_armed else 'DISARMED'}")
            logging.info("모든 연결 과정이 완료되었습니다. 이제 모터를 제어할 준비가 되었습니다.")
            return # 성공적으로 완료되면 함수 종료
    except asyncio.TimeoutError:
        logging.error("연결 또는 텔레메트리 수신 시간 초과!")
        logging.error("연결은 되었으나 FC가 텔레메트리 데이터를 보내지 않는 것일 수 있습니다.")
        logging.error("FC를 재부팅하고 ArduPilot 설정을 다시 확인해 보세요.")

    except Exception as e:
        logging.error(f"예상치 못한 오류 발생: {e}")
if name == "main":
    asyncio.run(run())
이 코드를 바탕으로 드론의 방향제어를 추가하고싶어 하나 파일이 아니라 방향제어 파일 하나 통신코드 하나 이렇게 분리해서 작성해줘

편집
드론 제어를 위한 코드를 두 개의 파일로 분리하여 작성해드리겠습니다. 통신 관리 파일과 방향 제어 파일로 나누어 모듈화된 구조로 만들겠습니다.


drone_connection.py - 드론 통신 관리 모듈
코드 

drone_control.py - 드론 방향 제어 모듈
코드 
드론 제어를 위한 두 개의 파일을 작성했습니다:

📁 파일 구조
1. drone_connection.py - 통신 관리 모듈
드론과의 연결 설정 및 관리
기본적인 드론 동작 (시동, 이륙, 착륙 등)
텔레메트리 데이터 수집
연결 상태 모니터링
2. drone_control.py - 방향 제어 모듈
6축 방향 이동 제어 (전/후/좌/우/상/하)
Yaw 회전 제어
GPS 좌표 이동
Offboard 모드 제어
속도 기반 제어
🎮 주요 기능
DroneConnection 클래스
connect(): 드론 연결
arm()/disarm(): 시동 제어
takeoff()/land(): 이착륙
get_telemetry_data(): 센서 데이터 수집
DroneController 클래스
move_forward/backward/left/right(): 수평 이동
move_up/down(): 수직 이동
rotate_yaw(): 좌우 회전
go_to_location(): GPS 좌표 이동
set_velocity_body(): 기체 좌표계 속도 제어
start/stop_offboard_mode(): 정밀 제어 모드
💡 사용 방법
python
# 간단한 사용 예제
from drone_connection import DroneConnection
from drone_control import DroneController

# 연결
connection = DroneConnection()
await connection.connect()

# 컨트롤러 생성
controller = DroneController(connection)

# 이륙 후 제어
await connection.takeoff()
await controller.move_forward(2.0)  # 2m 전진
await controller.rotate_yaw(90)      # 90도 회전
⚠️ 주의사항
Offboard 모드: 정밀한 속도 제어를 위해서는 Offboard 모드 활성화가 필요합니다
좌표계: NED(North-East-Down) 좌표계를 사용합니다
안전: 실제 드론 테스트 시 충분한 공간과 안전 장비를 확보하세요
GPS: GPS 기반 이동은 GPS 신호가 양호한 실외에서만 사용하세요
이 구조로 드론 제어 코드를 모듈화하여 유지보수와 확장이 용이하도록 설계했습니다.




위의 코드를 바탕으로 시동 후 이륙 및 직진하는 예제 코드를 알려줘

편집
시동 후 이륙하고 직진하는 간단한 예제 코드를 작성해드리겠습니다.


example_takeoff_forward.py - 이륙 및 직진 예제
코드 
시동 후 이륙 및 직진하는 예제 코드를 작성했습니다. 두 가지 버전을 포함했습니다:

📌 주요 기능
1. 간단한 비행 (simple_flight)
기본적인 비행 시퀀스를 수행합니다:

드론 연결 → 시동 → 2m 이륙 → 3m 직진 → 착륙 → 시동 끄기
2. 고급 비행 (advanced_flight)
사각형 패턴으로 비행하는 예제:

2m씩 전진하며 90도씩 회전하여 사각형 경로 비행
🚀 실행 방법
bash
# 필요한 파일들이 같은 디렉토리에 있어야 합니다:
# - drone_connection.py
# - drone_control.py
# - example_takeoff_forward.py

python example_takeoff_forward.py
💡 코드 특징
안전 기능
연결 확인: 각 단계마다 성공 여부 확인
텔레메트리 체크: 배터리, GPS 상태 확인
사용자 확인: 비행 시작 전 Enter 키 대기
긴급 처리: KeyboardInterrupt 및 예외 발생 시 자동 착륙
단계별 피드백
각 동작마다 상태 메시지 출력
이모지를 활용한 시각적 피드백
진행 상황을 명확하게 표시
안정화 대기
시동 후 2초 대기
이륙 후 5초 대기 (고도 안정화)
각 동작 사이 1초 대기
⚠️ 실행 전 체크리스트
환경 확인
실내: 최소 5m x 5m x 3m 공간
실외: 바람이 적고 장애물이 없는 곳
드론 상태
배터리 50% 이상
프로펠러 정상 장착
FC 연결 상태 확인
안전 장비
프로펠러 가드 장착 권장
비상 정지 준비
파라미터 조정
python
# 필요시 속도와 거리 조정
await controller.move_forward(
    distance_m=1.0,  # 거리를 줄여서 테스트
    speed_m_s=0.5    # 속도를 낮춰서 안전하게
)
이 예제를 통해 기본적인 드론 제어를 익히고, 점진적으로 복잡한 미션으로 확장할 수 있습니다!




간단하게 시동 위로 2M 상승 다시 하강 하고 종료하는 코드를 작성해줘

편집
시동 → 2m 상승 → 하강 → 종료하는 간단한 코드를 작성해드리겠습니다.


simple_up_down.py - 간단한 상승/하강 예제
코드 
시동 → 상승 → 하강 → 종료하는 간단한 코드를 작성했습니다.

🎯 동작 순서
드론 연결 - FC와 통신 연결
시동 걸기 - 모터 시동
2m 상승 - 0.5m/s 속도로 상승
3초 호버링 - 공중에서 대기
2m 하강 - 0.3m/s 속도로 안전하게 하강
시동 끄기 - 모터 정지
🚀 실행 방법
bash
python simple_up_down.py
⚙️ 파라미터 조정
필요에 따라 코드에서 다음 값들을 조정할 수 있습니다:

python
# 상승 높이와 속도 조정
await controller.move_up(
    distance_m=1.0,  # 1m로 줄이기
    speed_m_s=0.3    # 더 천천히
)

# 호버링 시간 조정
await asyncio.sleep(5)  # 5초로 늘리기

# 하강 속도 조정
await controller.move_down(
    distance_m=2.0,
    speed_m_s=0.2    # 더 안전하게
)
🛡️ 안전 기능
Ctrl+C 긴급 정지: 언제든 프로그램 중단 시 자동 하강
느린 하강 속도: 상승(0.5m/s)보다 하강(0.3m/s)을 더 천천히
단계별 확인: 각 단계 성공 여부 체크
사용자 확인: 시작 전 안전 확인 프롬프트
⚠️ 주의사항
실내 테스트 시
천장 높이 최소 3m 이상 확보
프로펠러 가드 장착 권장
실외 테스트 시
바람이 없는 날 진행
GPS 신호 양호한지 확인
첫 테스트는 낮은 높이로
python
# 처음엔 50cm 정도로 테스트
await controller.move_up(distance_m=0.5, speed_m_s=0.3)
이 간단한 코드로 드론의 기본적인 수직 이동을 안전하게 테스트할 수 있습니다!








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
