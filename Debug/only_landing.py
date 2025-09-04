# test_landing.py
import time
import cv2
from drone_controller import DroneController
from Landing import Landing
from pipeline import gstreamer_pipeline

def test_landing():
    """착륙 기능만 테스트하는 함수"""
    
    # 드론 시스템 객체 생성
    drone_system = DroneController()
    
    # 하단 카메라만 생성 (착륙용)
    pipeline1 = gstreamer_pipeline(sensor_id=1)
    cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)
    
    # 착륙 시스템 객체 생성
    landing = Landing(cap1, drone_system)
    
    try:
        # 드론 연결
        print("[테스트] 드론 연결 중...")
        if not drone_system.connect():
            print("[테스트] 드론 연결 실패")
            return
        
        # 시동
        print("[테스트] 시동 걸기...")
        if not drone_system.arm():
            print("[테스트] 시동 실패")
            return
        
        time.sleep(3)
        
        # 낮은 고도로 이륙 (2m)
        print("[테스트] 이륙 (2m)...")
        if not drone_system.takeoff(2.0):  # 수정: 2m로 낮게 이륙
            print("[테스트] 이륙 실패")
            drone_system.set_mode_land()
            return
        
        time.sleep(5)
        print("[테스트] 이륙 완료. 5초 후 착륙 시작...")
        
        # 착륙 테스트
        print("\n[테스트] === 착륙 테스트 시작 ===")
        landing.run()
        print("[테스트] === 착륙 테스트 완료 ===")
        
    except KeyboardInterrupt:
        print("\n[테스트] 사용자 중단 - 긴급 착륙")
        drone_system.set_mode_land()
        time.sleep(10)
        
    except Exception as e:
        print(f"[테스트] 오류 발생: {e}")
        if drone_system.is_armed:
            drone_system.set_mode_land()
            time.sleep(10)
    
    finally:
        cap1.release()
        cv2.destroyAllWindows()
        print("[테스트] 프로그램 종료")

if __name__ == "__main__":
    print("=" * 50)
    print("착륙 기능 테스트")
    print("빨간색 원형 마커를 준비하세요!")
    print("=" * 50)
    
    response = input("준비되셨나요? (y/n): ")
    if response.lower() == 'y':
        test_landing()
    else:
        print("테스트 취소")
