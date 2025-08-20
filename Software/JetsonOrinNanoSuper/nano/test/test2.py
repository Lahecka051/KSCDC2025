# Jetson Orin Nano (드론)용 실제 임무 수행 코드 (좌표/사진 동시 전송)
# PC로부터 경로를 받아 순찰, 탐지, 촬영 후 좌표와 사진을 함께 보고합니다.

import socket
import threading
import time
import json
import os
# import cv2
# from dronekit import connect, VehicleMode

# --- 설정 변수 ---
CONTROLLER_PC_IP = '지니컴.local'  # 관제 센터 PC의 IP 주소

# --- 시뮬레이션용 가상 객체 및 함수 (이전과 동일) ---
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

# --- 기능 수정: 화재 보고 전송 함수 ---
def send_fire_report(coordinates, image_path):
    """PC 관제 센터로 화재 보고(좌표, 이미지)를 전송합니다."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROLLER_PC_IP, 4000))
            
            # 1. 헤더 생성 (좌표, 이미지 크기 정보 포함)
            header = {
                "coordinates": coordinates,
                "image_size": os.path.getsize(image_path)
            }
            header_json = json.dumps(header)
            
            # 헤더를 1024바이트의 고정 크기로 만듦 (빈 공간은 공백으로 채움)
            header_bytes = header_json.encode('utf-8').ljust(1024, b' ')

            # 2. 헤더 전송
            s.sendall(header_bytes)
            
            # 3. 이미지 데이터 전송
            s.recv(1024) # PC로부터 "HEADER_OK" 신호 수신 대기
            with open(image_path, 'rb') as f:
                s.sendall(f.read())
            
            print(f"화재 보고 전송 성공!")

    except Exception as e:
        print(f"[오류] PC로 보고 전송 실패: {e}")

# --- 임무 수행 로직 (수정) ---
def run_mission(path):
    if not path: return
    print("\n--- 전송받은 경로로 임무 시작 ---")

    # 1. 순찰 (Patrolling)
    print("[상태: 순찰]")
    # ... (순찰 로직) ...
    for i, waypoint in enumerate(path):
        print(f"경로점 {i+1} ({waypoint['lat']:.4f}, {waypoint['lon']:.4f}) 으로 이동 중... (정면 카메라 활성)")
        time.sleep(4)
    
    # 마지막 지점 근처에서 화재를 발견했다고 가정
    fire_detected = True
    print("!!! 화재 탐지 !!! 드론을 정지합니다.")

    if fire_detected:
        # 2. 위치 특정 (Targeting)
        print("\n[상태: 위치 특정]")
        time.sleep(2)
        fire_gps = path[-1] # 화재 GPS 좌표 계산 완료 (시뮬레이션)
        print(f"화재 중심 GPS 계산 완료: {fire_gps}")
        print(f"화재 지점 상공으로 이동합니다...")
        time.sleep(3)
        print("화재 지점 상공 도착.")

        # 3. 촬영 (Capturing)
        print("\n[상태: 촬영]")
        captured_image_path = capture_image(camera_index=1, filename="fire_scene.jpg")
        
        # 4. 복귀 (Returning)
        print("\n[상태: 복귀]")
        print("임무 완료. 스테이션으로 복귀합니다 (RTL).")
        time.sleep(10)
        print("스테이션 착륙 완료.")

        # 5. 보고 (Reporting) - 수정된 함수 호출
        print("\n[상태: 보고]")
        print("관제 센터로 좌표와 사진을 함께 전송합니다.")
        send_fire_report(fire_gps, captured_image_path)
    
    print("\n--- 모든 임무 절차 완료 ---")

# --- 메인 로직: PC로부터 순찰 명령 수신 대기 (이전과 동일) ---
def main():
    host = '0.0.0.0'; port = 3999
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port)); s.listen()
        print(f"Jetson 임무 대기 중... (Port: {port})")
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"\nPC({addr})로부터 연결됨. 경로 데이터 수신 중...")
                data = conn.recv(4096)
                if data:
                    try:
                        path_data = json.loads(data.decode('utf-8'))
                        mission_thread = threading.Thread(target=run_mission, args=(path_data,), daemon=True)
                        mission_thread.start()
                    except Exception as e:
                        print(f"경로 데이터 처리 중 오류 발생: {e}")

if __name__ == '__main__':
    main()
