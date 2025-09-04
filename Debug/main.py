# Jetson Orin Nano (드론)용 최종 코드
import socket
import threading
import time
import json
import os
import cv2

# 사용자가 만든 모듈
from DroneCommunicator import DroneCommunicator
from nonfire_patrol import Patrol  # 수정: nonfire_patrol 사용
# from Fire_detector import Fire_detector  # 수정: 제거
from drone_controller import DroneController
# from Fire_extinguishing import Fire_extinguishing  # 수정: 제거
from pipeline import gstreamer_pipeline
from Landing import Landing
from servo_control import Servo

# --- 드론 시스템 객체 생성 ---
drone_system = DroneController()

# --- 서보 객체 생성 ---
servo = Servo(servo_pin=33)

# --- 통신 모듈 객체 생성 ---
CONTROLLER_PC_HOSTNAME = 'AERO_17.local' 
communicator = DroneCommunicator(pc_hostname=CONTROLLER_PC_HOSTNAME)

# --- 카메라 객체 생성 ----
pipeline0 = gstreamer_pipeline(sensor_id=0)
pipeline1 = gstreamer_pipeline(sensor_id=1)
cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)

# --- 착륙 시스템 객체 생성 ---
landing = Landing(cap1, drone_system)

# --- 순찰 객체 생성 --- # 수정: fire_detector 없이 생성
patrol = Patrol(drone_system, landing, communicator)

# --- 메인 로직 : 단순 순찰 테스트 ---
def main():
    try:
        print("[메인] GPS 이동 테스트 모드")
        print("[메인] 화재 탐지: 비활성화")
        print("[메인] 착륙: 색상 기반 마커 탐지 사용")
        
        # 드론 연결
        if not drone_system.connect():
            print("[메인] 드론 연결 실패")
            return
        
        communicator.start_receiver()
        while True:
            data = communicator.receive_data()
            if data:
                try:
                    command_data = json.loads(data.decode('utf-8').strip())

                    if isinstance(command_data, dict) and command_data.get('type') == 'EXTINGUISH':
                        print("[메인] 소화 미션은 비활성화 상태")
                        continue
                        
                    elif isinstance(command_data, list):
                        print(f"[메인] GPS 경로 수신: {len(command_data)}개 지점")
                        mission_thread = threading.Thread(
                            target=patrol.run, 
                            args=(command_data,), 
                            daemon=True
                        )
                        mission_thread.start()
                    else:
                        print(f"[메인] 알 수 없는 명령: {command_data}")

                except Exception as e:
                    print(f"[메인] 오류: {e}")
                    
    except KeyboardInterrupt:
        print("\n[메인] 프로그램 종료 중...")
        if drone_system.is_armed:
            print("[메인] 긴급 착륙...")
            drone_system.set_mode_land()
            time.sleep(5)
            drone_system.disarm()
        
    finally:
        servo.cleanup()
        cap0.release()
        cap1.release()
        cv2.destroyAllWindows()
        print("[메인] 종료 완료")

if __name__ == '__main__':
    main()

# --- 메인 로직 : 화재 탐지 및 소화 탑재 ---
# def main():
#     try:
#         # 수정: 드론 연결
#         if not drone_system.connect():
#             print("[메인] 드론 연결 실패")
#             return
        
#         communicator.start_receiver()
#         while True:
#             data = communicator.receive_data()
#             if data:
#                 try:
#                     command_data = json.loads(data.decode('utf-8').strip())

#                     if isinstance(command_data, dict) and command_data.get('type') == 'EXTINGUISH':
#                         target = command_data.get('target')
#                         mission_thread = threading.Thread(target=fire_extinguishing.run, args=(target,), daemon=True)
#                         mission_thread.start()
#                     elif isinstance(command_data, list):
#                         mission_thread = threading.Thread(target=patrol.run, args=(command_data,), daemon=True)
#                         mission_thread.start()
#                     else:
#                         print(f"알 수 없는 형식의 명령 수신: {command_data}")

#                 except Exception as e:
#                     print(f"명령 처리 중 오류 발생: {e}")
                    
#     except KeyboardInterrupt:
#         print("\n[메인] 프로그램 종료 중...")
        
#     finally:
#         # 수정: 프로그램 종료 시 정리
#         servo.cleanup()
#         cap0.release()
#         cap1.release()
#         cv2.destroyAllWindows()
#         print("[메인] 종료 완료")

# if __name__ == '__main__':
#     main()