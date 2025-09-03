# Jetson Orin Nano (드론)용 최종 코드
import socket
import threading
import time
import json
import os
import cv2  # 수정: cv2 import 추가

# 사용자가 만든 모듈
from DroneCommunicator import DroneCommunicator  # 수정: 파일명에 맞게 변경
from patrol import Patrol  # 수정: 클래스명 대문자로
from Fire_detector import Fire_detector  # 수정: 파일명에 맞게 변경
from drone_controller import DroneController  # 수정: 드론 컨트롤러 import 추가
from Fire_extinguishing import Fire_extinguishing  # 수정: 더 완성된 버전 사용
from pipeline import gstreamer_pipeline  # 수정: pipeline import 추가
from Landing import Landing  # 수정: 실제 Landing 모듈 import
# from servo import Servo  # Servo 모듈이 없으므로 주석처리

# --- 임시 Servo 클래스 정의 --- # 수정: Servo 클래스가 없어서 임시 정의
class Servo:
    def __init__(self):
        self.angle = 90
    
    def set_angle(self, angle):
        self.angle = angle
        print(f"[서보] 각도 설정: {angle}도")

# --- 드론 시스템 객체 생성 --- # 수정: DroneController 객체 생성
drone_system = DroneController()

# --- 서보 객체 생성 --- # 수정: Servo 객체 생성
servo = Servo()

# --- 통신 모듈 객체 생성 ---
# ❗️❗️❗️PC의 실제 이름으로 반드시 변경해주세요❗️❗️❗️
CONTROLLER_PC_HOSTNAME = 'Your-PC-Name.local' 
communicator = DroneCommunicator(pc_hostname=CONTROLLER_PC_HOSTNAME)

# --- 카메라 객체 생성 ----
pipeline0 = gstreamer_pipeline(sensor_id=0)
pipeline1 = gstreamer_pipeline(sensor_id=1)
cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)

# --- 화재탐지 객체 생성 ---
fire_detector = Fire_detector(cap0, cap1)  # 수정: 변수명 오타 수정

# --- 착륙 시스템 객체 생성 ---
landing = Landing(cap1, drone_system)  # 실제 Landing 클래스 사용

# --- 순찰 객체 생성 ---
patrol = Patrol(drone_system, fire_detector, landing, communicator)

# --- 소화볼 투하 객체 생성 ---
fire_extinguishing = Fire_extinguishing(drone_system, fire_detector, landing, communicator, servo)

# --- 메인 로직: PC로부터 명령 수신 대기 ---
def main():
    # 수정: 드론 연결
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
                    target = command_data.get('target')
                    mission_thread = threading.Thread(target=fire_extinguishing.run, args=(target,), daemon=True)
                    mission_thread.start()
                elif isinstance(command_data, list):
                    mission_thread = threading.Thread(target=patrol.run, args=(command_data,), daemon=True)
                    mission_thread.start()
                else:
                    print(f"알 수 없는 형식의 명령 수신: {command_data}")

            except Exception as e:
                print(f"명령 처리 중 오류 발생: {e}")

if __name__ == '__main__':
    main()