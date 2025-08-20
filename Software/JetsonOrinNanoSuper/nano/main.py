# Jetson Orin Nano (드론)용 최종 코드
import socket
import threading
import time
import json
import os

# 사용자가 만든 코드
from drone_communications import DroneCommunicator


# --- 설정 변수 ---
# ❗️❗️❗️PC의 실제 이름으로 반드시 변경해주세요❗️❗️❗️
CONTROLLER_PC_HOSTNAME = 'Your-PC-Name.local' 

# --- 통신 모듈 객체 생성 ---
communicator = DroneCommunicator(pc_hostname=CONTROLLER_PC_HOSTNAME)


# --- 메인 로직: PC로부터 명령 수신 대기 ---
def main():
    host = '0.0.0.0'; port = 3999
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port)); s.listen()
        print(f"Jetson 임무 대기 중... (Port: {port})")
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"\nPC({addr})로부터 연결됨. 명령 수신 중...")
                data = conn.recv(4096)
                if data:
                    try:
                        command_data = json.loads(data.decode('utf-8'))
                        
                        if isinstance(command_data, dict) and command_data.get('type') == 'EXTINGUISH':
                            target = command_data.get('target')
                            mission_thread = threading.Thread(target=run_extinguish_mission, args=(target,), daemon=True)   #화제 진압 알고리즘 run_extinguish_mission에 넣기
                            mission_thread.start()
                        elif isinstance(command_data, list):
                            mission_thread = threading.Thread(target=run_mission, args=(command_data,), daemon=True)    #순찰 알고리즘 run_mission에 넣기
                            mission_thread.start()
                        else:
                            print(f"알 수 없는 형식의 명령 수신: {command_data}")

                    except Exception as e:
                        print(f"명령 처리 중 오류 발생: {e}")

if __name__ == '__main__':
    main()
