# PatrolManager.py
import time
import socket
import threading
from Object_Data import Object_Data
from Landing import Landing
import cv2
import os

class PatrolManager:
    def __init__(self, upper_cam_index=0, lower_cam_index=1, marker_path="/home/kscdc2025/Marker.png"):
        # 상부 카메라: 화재 탐지
        self.upper_cam = cv2.VideoCapture(upper_cam_index)
        # 하부 카메라: 착륙 및 화재 사진 촬영
        self.lower_cam = cv2.VideoCapture(lower_cam_index)

        # 객체 감지 클래스
        self.object_data = Object_Data(self.upper_cam)
        # 착륙 클래스, marker_path 명시적으로 전달
        self.landing = Landing(self.lower_cam, marker_path=marker_path)

        # 드론 상태
        self.fire_detected = False
        self.fire_image_path = None
        self.fire_coordinates = None

        # 관제 센터 IP, 포트
        self.control_center_ip = "192.168.0.10"  # 예시
        self.control_center_port = 4000

    def patrol_loop(self):
        """순찰 중 화재 감지 및 처리"""
        while True:
            detected, image_path, command = self.object_data.detect_fire()
            if detected:
                self.fire_detected = True
                self.fire_image_path = image_path
                self.fire_coordinates = command  # 또는 실제 GPS 좌표 변환 후 저장

                print("화재 위치로 이동 중...")
                self.move_to_fire(command)

                print("화재 상공 도착, 하부 카메라 촬영...")
                self.capture_fire()

                print("순찰 경로로 복귀 중...")
                self.return_to_station()

                print("드론 착륙 중 (마커 추적)...")
                self.perform_landing()

                print("와이파이 연결 시 관제 센터로 화재 이미지 전송 대기...")
                self.send_fire_report_when_wifi_ready()

                # 한 번 처리 후 상태 초기화
                self.fire_detected = False
                self.fire_image_path = None
                self.fire_coordinates = None

            time.sleep(0.1)

    def move_to_fire(self, command):
        """화재 위치 위로 이동 (상공 호버링 유지)"""
        print(f"화재 위치로 이동 명령: {command}")
        # 실제 드론 명령 전송 코드 포함 가능
        time.sleep(2)

    def capture_fire(self):
        """하부 카메라로 화재 현장 촬영"""
        ret, frame = self.lower_cam.read()
        if ret:
            save_dir = "fire_captures"
            os.makedirs(save_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(save_dir, f"fire_{timestamp}.jpg")
            cv2.imwrite(path, frame)
            self.fire_image_path = path
            print(f"화재 사진 저장 완료: {path}")

    def return_to_station(self):
        """순찰 경로 따라 드론 스테이션으로 복귀"""
        print("스테이션으로 복귀 중...")
        time.sleep(2)

    def perform_landing(self):
        """하부 카메라로 Marker.png 인식 후 정밀 착륙"""
        print("착륙 시작...")
        landing_complete = False
        while not landing_complete:
            ret, frame = self.lower_cam.read()
            if not ret:
                continue
            cmd, marker_center, debug_frame = self.landing.process_frame(frame)
            if cmd is not None and cmd[1] == "stop":
                landing_complete = True
                print("마커 인식 완료, 착륙 성공!")
            # 실제 드론 명령 전송 코드 포함 가능
            cv2.imshow("Landing Detection", debug_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

    def send_fire_report_when_wifi_ready(self):
        """착륙 후 와이파이 연결되면 관제 센터로 화재 이미지 전송"""
        if self.fire_image_path is None:
            return

        # 와이파이 연결 확인 (예시, 실제 구현 필요)
        wifi_connected = True
        while not wifi_connected:
            print("와이파이 연결 대기 중...")
            time.sleep(2)
            wifi_connected = True  # 실제 연결 상태 확인

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.control_center_ip, self.control_center_port))

                # 이미지 읽기
                with open(self.fire_image_path, "rb") as f:
                    img_data = f.read()
                header = {
                    "coordinates": self.fire_coordinates,
                    "image_size": len(img_data)
                }
                s.sendall(bytes(json.dumps(header).ljust(1024), "utf-8"))
                ack = s.recv(1024)
                if ack == b"HEADER_OK":
                    s.sendall(img_data)
                    print("화재 이미지 관제 센터 전송 완료!")
        except Exception as e:
            print(f"화재 이미지 전송 오류: {e}")

if __name__ == "__main__":
    patrol_manager = PatrolManager(marker_path="/home/kscdc2025/Marker.png")
    patrol_manager.patrol_loop()
