import time
import cv2
import json
import socket
from Object_Data import Object_Data
from Landing import Landing

class PatrolManager:
    def __init__(self, upper_cam_index=0, lower_cam_index=1, marker_path="/home/kscdc2025/Marker.png"):
        self.upper_cam = cv2.VideoCapture(upper_cam_index)
        self.lower_cam = cv2.VideoCapture(lower_cam_index)
        self.upper_od = Object_Data(self.upper_cam, mode="upper")
        self.lower_od = Object_Data(self.lower_cam, mode="lower")
        self.landing = Landing(self.lower_cam, marker_path=marker_path)
        self.fire_coordinates = None
        self.fire_image_path = None
        self.control_center_ip = "192.168.0.10"
        self.control_center_port = 4000

    def patrol_loop(self):
        while True:
            detected, coords, frame = self.upper_od.detect_fire_upper()
            if detected:
                print("전방 화재 탐지 → 5초 호버링 확인")
                hover_start = time.time()
                fire_confirmed = False
                while time.time() - hover_start < 5:
                    detected2, _, _ = self.upper_od.detect_fire_upper()
                    if detected2:
                        fire_confirmed = True
                        break
                    time.sleep(0.1)

                if fire_confirmed:
                    print("화재 확실 → 상공 이동")
                    self.fire_coordinates = coords
                    time.sleep(2)  # 이동 시뮬레이션

                    print("하부 카메라 중앙 정렬 → 사진 촬영")
                    captured_path = self.lower_od.detect_and_align_fire(timeout=5)
                    if captured_path:
                        self.fire_image_path = captured_path

                    print("Landing 실행 → 착륙 수행")
                    self.landing.run()

                    self.send_fire_report()
            time.sleep(0.1)

    def send_fire_report(self):
        if not self.fire_image_path:
            return
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.control_center_ip, self.control_center_port))
                with open(self.fire_image_path,"rb") as f:
                    img_data = f.read()
                header = {"coordinates":self.fire_coordinates,"image_size":len(img_data)}
                s.sendall(bytes(json.dumps(header).ljust(1024),"utf-8"))
                ack = s.recv(1024)
                if ack == b"HEADER_OK":
                    s.sendall(img_data)
                    print("화재 이미지 전송 완료")
        except Exception as e:
            print(f"화재 전송 오류: {e}")

# ------------------------
# 실행 예시
# ------------------------
if __name__=="__main__":
    pm = PatrolManager()
    pm.patrol_loop()
