# Jetson Orin Nano (드론)용 최종 코드
import socket
import threading
import time
import json
import os

# 사용자가 만든 모듈
from drone_communications import DroneCommunicator
from patrol import patrol
from Fire_detector import Fire_detector
from Landing import Landing

# --- 통신 모듈 객체 생성 ---
# ❗️❗️❗️PC의 실제 이름으로 반드시 변경해주세요❗️❗️❗️
CONTROLLER_PC_HOSTNAME = 'Your-PC-Name.local' 
communicator = DroneCommunicator(pc_hostname=CONTROLLER_PC_HOSTNAME)

# --- 카메라 객체 생성 ----
pipeline0 = gstreamer_pipeline(sensor_id=0)
pipeline1 = gstreamer_pipeline(sensor_id=1)
cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)

# --- 화제탐지 객체 생성 ---
fire_dector = Fire_detector(cap0,cap1)

# --- 착륙 시스템 객체 생성 ---
#Landing(cap, drone_system: IntegratedDroneSystem, marker_path="/home/kscdc2025/Marker.png")
landing = Landing(cap1, drone_system)

# --- 순찰 객체 생성 ---
#Patrol(drone_system: IntegratedDroneSystem, fire_detector, landing, communicator)
patrol = Patrol(drone_system: IntegratedDroneSystem, fire_detector, landing, communicator)

# --- 메인 로직: PC로부터 명령 수신 대기 ---
def main():
    communicator.start_receiver()
    while True:
        data = communicator.receive_data()
        if data:
            try:
                command_data = json.loads(data.decode('utf-8'))

                if isinstance(command_data, dict) and command_data.get('type') == 'EXTINGUISH':
                    target = command_data.get('target')
                    mission_thread = threading.Thread(target=run_extinguish_mission, args=(target,), daemon=True)   #화제 진압 알고리즘 run_extinguish_mission에 넣기
                    mission_thread.start()
                elif isinstance(command_data, list):
                    mission_thread = threading.Thread(target=patrol.run, args=(command_data,), daemon=True)    #순찰 알고리즘 run_mission에 넣기
                    mission_thread.start()
                else:
                    print(f"알 수 없는 형식의 명령 수신: {command_data}")

            except Exception as e:
                print(f"명령 처리 중 오류 발생: {e}")

if __name__ == '__main__':
    main()

