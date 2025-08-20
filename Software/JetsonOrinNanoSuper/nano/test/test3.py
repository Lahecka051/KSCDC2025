# Jetson Orin Nano (드론)용 최종 코드
import socket
import threading
import time
import json
import os
# import cv2
# from dronekit import connect, VehicleMode

# --- 설정 변수 ---
# ❗️❗️❗️PC의 실제 이름으로 반드시 변경해주세요❗️❗️❗️
# (PC의 CMD창에서 hostname 입력하여 확인)
CONTROLLER_PC_HOSTNAME = 'Your-PC-Name.local' 

# --- 시뮬레이션용 가상 객체 및 함수 ---
class FakeVehicle:
    def __init__(self):
        self.location = type('obj', (object,), {'global_relative_frame': type('obj', (object,), {'lat': 37.4, 'lon': 127.1})})()
        self.mode = type('obj', (object,), {'name': 'LOITER'})
    def close(self): print("가상 드론 연결 해제.")
vehicle = FakeVehicle()

def capture_image(camera_index, filename="capture.jpg"):
    print(f"카메라 {camera_index} (0:정면, 1:하방)로 사진 촬영 시도...")
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new('RGB', (640, 480), color='darkgray')
    d = ImageDraw.Draw(img)
    text = f"CAM {camera_index} Capture\n{time.ctime()}"
    d.text((10, 10), text, fill=(255, 255, 0))
    if camera_index == 1:
        d.ellipse([(280, 200), (360, 280)], fill='red', outline='orange')
    img.save(filename)
    print(f"'{filename}'으로 사진 저장 완료.")
    return filename

def send_fire_report(coordinates, image_path):
    """PC 관제 센터로 화재 보고(좌표, 이미지)를 전송합니다."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROLLER_PC_HOSTNAME, 4000))
            
            header = {
                "type": "FIRE_REPORT",
                "coordinates": coordinates,
                "image_size": os.path.getsize(image_path)
            }
            header_json = json.dumps(header)
            header_bytes = header_json.encode('utf-8').ljust(1024, b' ')

            s.sendall(header_bytes)
            s.recv(1024) # HEADER_OK
            with open(image_path, 'rb') as f:
                s.sendall(f.read())
            
            print(f"화재 보고 전송 성공!")
    except Exception as e:
        print(f"[오류] PC로 보고 전송 실패: {e}")

def send_status_update(status_message):
    """PC 관제 센터로 간단한 상태 메시지를 전송합니다."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROLLER_PC_HOSTNAME, 4000))
            
            message_data = {
                "type": "STATUS_UPDATE",
                "status": status_message
            }
            message_bytes = json.dumps(message_data).encode('utf-8')
            
            s.sendall(message_bytes)
            print(f"상태 보고 전송 성공: {status_message}")
    except Exception as e:
        print(f"[오류] PC로 상태 보고 전송 실패: {e}")

# --- 순찰 임무 수행 로직 ---
def run_mission(path):
    if not path: return
    print("\n--- 전송받은 경로로 순찰 임무 시작 ---")
    print("[상태: 순찰]")
    for i, waypoint in enumerate(path):
        print(f"경로점 {i+1} ({waypoint['lat']:.4f}, {waypoint['lon']:.4f}) 으로 이동 중...")
        time.sleep(4)
    
    fire_detected = True
    print("!!! 화재 탐지 !!! 드론을 정지합니다.")

    if fire_detected:
        print("\n[상태: 위치 특정]")
        time.sleep(2)
        fire_gps = path[-1]
        print(f"화재 중심 GPS 계산 완료: {fire_gps}")
        
        print("\n[상태: 촬영]")
        captured_image_path = capture_image(camera_index=1, filename="fire_scene.jpg")
        
        print("\n[상태: 복귀]")
        print("임무 완료. 스테이션으로 복귀합니다 (RTL).")
        time.sleep(10)
        print("스테이션 착륙 완료.")

        print("\n[상태: 보고]")
        print("관제 센터로 좌표와 사진을 함께 전송합니다.")
        send_fire_report(fire_gps, captured_image_path)
    
    print("\n--- 순찰 임무 절차 완료 ---")

# --- 화재 진압 임무 수행 로직 ---
def run_extinguish_mission(target_coords):
    print("\n--- 화재 진압 임무 수신 ---")
    lat, lon = target_coords['lat'], target_coords['lon']
    
    print(f"목표 지점({lat:.4f}, {lon:.4f})으로 출동합니다...")
    time.sleep(7)
    
    print("목표 지점 상공 도착. 소화볼 투척 준비...")
    time.sleep(2)
    
    print("!!! 소화볼 투척 완료 !!!")
    
    print("임무 완료. 스테이션으로 복귀합니다.")
    time.sleep(7)
    
    print("스테이션 착륙 완료. 임무 대기 모드로 전환합니다.")
    send_status_update("EXTINGUISH_COMPLETE")
    
    print("\n--- 진압 임무 절차 완료 ---")

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
                            mission_thread = threading.Thread(target=run_extinguish_mission, args=(target,), daemon=True)
                            mission_thread.start()
                        elif isinstance(command_data, list):
                            mission_thread = threading.Thread(target=run_mission, args=(command_data,), daemon=True)
                            mission_thread.start()
                        else:
                            print(f"알 수 없는 형식의 명령 수신: {command_data}")

                    except Exception as e:
                        print(f"명령 처리 중 오류 발생: {e}")

if __name__ == '__main__':
    main()
