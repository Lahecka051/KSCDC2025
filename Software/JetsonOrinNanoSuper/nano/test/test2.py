# Jetson Orin Nano (드론)용 실제 임무 수행 코드
# PC로부터 경로를 받아 순찰, 탐지, 촬영, 복귀, 보고 임무를 수행합니다.

import socket
import threading
import time
import json
import os
# import cv2  # 실제 카메라와 CV 모델 사용 시 주석 해제
# from dronekit import connect, VehicleMode # 실제 FC 연결 시 주석 해제

# --- 설정 변수 ---
CONTROLLER_PC_IP = '192.168.83.128'  # 관제 센터 PC의 IP 주소

# --- 시뮬레이션용 가상 객체 및 함수 ---
# (이전의 FC 없이 테스트하던 코드를 활용하여 시뮬레이션 환경 구성)
class FakeVehicle:
    def __init__(self):
        self.location = type('obj', (object,), {'global_relative_frame': type('obj', (object,), {'lat': 37.4, 'lon': 127.1})})()
        self.mode = type('obj', (object,), {'name': 'LOITER'})
    def close(self): print("가상 드론 연결 해제.")
# 실제 FC 연결 시: vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)
vehicle = FakeVehicle()

def capture_image(camera_index, filename="capture.jpg"):
    """가상으로 카메라에서 이미지를 캡처하고 파일로 저장합니다."""
    print(f"카메라 {camera_index} (0:정면, 1:하방)로 사진 촬영 시도...")
    # 실제 코드:
    # cap = cv2.VideoCapture(camera_index)
    # ret, frame = cap.read()
    # if ret: cv2.imwrite(filename, frame)
    # cap.release()

    # 시뮬레이션 코드: 가짜 이미지 파일 생성
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new('RGB', (640, 480), color = 'darkgray')
    d = ImageDraw.Draw(img)
    text = f"CAM {camera_index} Capture\n{time.ctime()}"
    d.text((10,10), text, fill=(255,255,0))
    if camera_index == 1: # 하방 카메라인 경우 화재 모양 추가
        d.ellipse([(280, 200), (360, 280)], fill='red', outline='orange')
    img.save(filename)
    print(f"'{filename}'으로 사진 저장 완료.")
    return filename

# --- 통신 함수 ---
def send_report_to_pc(report_type, data):
    """PC 관제 센터로 보고(경보 또는 이미지)를 전송합니다."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROLLER_PC_IP, 4000))
            
            # 1. 메시지 타입 전송
            s.sendall(report_type.encode('utf-8'))
            s.recv(1024) # OK 수신 대기

            if report_type == "ALERT":
                s.sendall(data.encode('utf-8'))
            elif report_type == "IMAGE":
                # 2. 이미지 크기 전송
                img_size = os.path.getsize(data)
                s.sendall(str(img_size).encode('utf-8'))
                s.recv(1024) # OK 수신 대기

                # 3. 이미지 데이터 전송
                with open(data, 'rb') as f:
                    s.sendall(f.read())
            
            print(f"'{report_type}' 보고 전송 성공!")

    except Exception as e:
        print(f"[오류] PC로 보고 전송 실패: {e}")

# --- 임무 수행 로직 ---
def run_mission(path):
    if not path: return
    print("\n--- 전송받은 경로로 임무 시작 ---")

    # 1. 순찰 (Patrolling)
    print("[상태: 순찰]")
    fire_detected = False
    for i, waypoint in enumerate(path):
        print(f"경로점 {i+1} ({waypoint['lat']:.4f}, {waypoint['lon']:.4f}) 으로 이동 중... (정면 카메라 활성)")
        time.sleep(4)
        # 이 루프 안에서 실제로는 정면 카메라로 계속 화재를 탐지해야 함
    
    # 마지막 지점 근처에서 화재를 발견했다고 가정
    fire_detected = True
    print("!!! 화재 탐지 !!! 드론을 정지합니다.")
    # 실제 코드: vehicle.mode = VehicleMode("LOITER")

    if fire_detected:
        # 2. 위치 특정 (Targeting)
        print("\n[상태: 위치 특정]")
        print("정면 카메라로 화재 중심점의 GPS 좌표 계산 중...")
        time.sleep(2)
        # 화재의 실제 GPS 좌표를 계산했다고 가정 (마지막 웨이포인트 위치로 시뮬레이션)
        fire_gps = path[-1]
        print(f"화재 중심 GPS 계산 완료: {fire_gps}")
        
        print(f"화재 지점 상공({fire_gps['lat']:.4f}, {fire_gps['lon']:.4f})으로 이동합니다...")
        time.sleep(3)
        print("화재 지점 상공 도착.")

        # 3. 촬영 (Capturing)
        print("\n[상태: 촬영]")
        print("하방 카메라로 전환합니다.")
        captured_image_path = capture_image(camera_index=1, filename="fire_scene.jpg")
        
        # 4. 복귀 (Returning)
        print("\n[상태: 복귀]")
        print("임무 완료. 스테이션으로 복귀합니다 (RTL).")
        # 실제 코드: vehicle.mode = VehicleMode("RTL")
        time.sleep(10) # 복귀 시간 시뮬레이션
        print("스테이션 착륙 완료.")

        # 5. 보고 (Reporting)
        print("\n[상태: 보고]")
        print("관제 센터로 촬영된 사진을 전송합니다.")
        send_report_to_pc("IMAGE", captured_image_path)
    
    print("\n--- 모든 임무 절차 완료 ---")

# --- 메인 로직: PC로부터 순찰 명령 수신 대기 ---
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
