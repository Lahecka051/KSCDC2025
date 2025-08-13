# Jetson Orin Nano (드론)용 시뮬레이션 코드 (FC 연결 불필요)
# 이 스크립트는 FC 없이 Jetson 보드의 로직(화재 탐지, 데이터 전송)을 테스트하기 위해 사용됩니다.

import time
import socket
# import cv2 # 실제 CV 모델을 테스트할 때 주석 해제

# --- 설정 변수 ---
CONTROLLER_PC_IP = '192.168.83.119' # 관제 센터 PC의 IP 주소 (실제 환경에 맞게 변경)
CONTROLLER_PC_PORT = 9999

# --- 가상 객체 (Dronekit의 Vehicle 객체를 흉내) ---
class FakeVehicle:
    """
    dronekit의 Vehicle 객체를 흉내 내는 가상 클래스입니다.
    FC 없이도 코드가 오류 없이 실행되도록 돕습니다.
    """
    def __init__(self):
        # 가상의 드론 상태를 설정합니다.
        self.mode = self._FakeMode("AUTO")
        self.armed = True
        self.location = self._FakeLocation()
        print("가상 드론 객체 생성 완료. 모드: AUTO, 상태: Armed")

    class _FakeMode:
        def __init__(self, name):
            self.name = name

    class _FakeLocation:
        def __init__(self):
            self.global_relative_frame = self._FakeFrame()
        
        class _FakeFrame:
            def __init__(self):
                # 시뮬레이션에서 사용할 고정된 가상 위치 (예: 판교)
                self.lat = 37.4021
                self.lon = 127.1089
                self.alt = 20.0 # 가상 고도

    def close(self):
        print("가상 드론 연결 해제.")

# --- 화재 탐지 로직 (시뮬레이션) ---
# 실제로는 이 함수에 OpenCV와 YOLO 모델을 이용한 탐지 코드가 들어갑니다.
def detect_fire(vehicle):
    """
    카메라 영상을 분석하여 화재를 탐지하는 함수.
    이 예제에서는 15초 후 화재를 탐지했다고 가정합니다.
    """
    if vehicle.mode.name == 'AUTO':
        if not hasattr(detect_fire, "start_time"):
            detect_fire.start_time = time.time() # 함수 첫 호출 시 시간 기록
        
        if time.time() - detect_fire.start_time > 15:
            print("!!! 화재 탐지 (Jetson 시뮬레이션) !!!")
            return True
            
    return False

# --- 메인 로직 ---
def main():
    # 1. 가상 드론 객체 생성 (FC 연결 대체)
    print("FC 연결 시퀀스를 대체하여 가상 드론을 생성합니다.")
    vehicle = FakeVehicle()

    try:
        while True:
            # 2. 화재 탐지 로직 실행
            is_fire_detected = detect_fire(vehicle)

            if is_fire_detected:
                # 3. 화재 탐지 시 처리
                print("화재 탐지! 보고 절차 시작.")

                # 가상 위치(GPS) 저장
                fire_location = vehicle.location.global_relative_frame
                print(f"화재 발생 위치 저장: lat={fire_location.lat}, lon={fire_location.lon}")

                # 가상 착륙 대기
                print("가상 착륙 시퀀스 진행... (5초 대기)")
                time.sleep(5)
                vehicle.armed = False # 가상 시동 끄기
                print("가상 착륙 완료. 데이터 전송을 시도합니다.")

                # 4. 관제 센터 PC로 데이터 전송
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((CONTROLLER_PC_IP, CONTROLLER_PC_PORT))
                        message = f"{fire_location.lat},{fire_location.lon}"
                        s.sendall(message.encode('utf-8'))
                        print("화재 위치 데이터 전송 성공!")
                except Exception as e:
                    print(f"데이터 전송 실패: {e}")
                
                # 임무 완료 후 프로그램 종료
                break
            
            print("가상 순찰 비행 중...")
            time.sleep(1)

    except KeyboardInterrupt:
        print("사용자에 의해 프로그램이 중지되었습니다.")
    finally:
        # 5. 가상 연결 종료
        vehicle.close()


if __name__ == '__main__':
    main()
