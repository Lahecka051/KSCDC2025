import socket
import json
import time
import cv2
from Object_Data import Object_Data
from Landing import Landing


class PatrolManager:
    def __init__(self, upper_cam_index=0, lower_cam_index=1, marker_path="/home/kscdc2025/Marker.png"):
        self.upper_cam = cv2.VideoCapture(upper_cam_index)
        self.lower_cam = cv2.VideoCapture(lower_cam_index)
        self.upper_od = Object_Data(self.upper_cam,"upper")
        self.lower_od = Object_Data(self.lower_cam,"lower")
        self.landing = Landing(self.lower_cam, marker_path)
        self.saved_fire_data = []  # 착륙 전 로컬 저장
        self.control_center_ip = "192.168.0.10"
        self.control_center_port = 4000

    def patrol_loop(self, mission_waypoints):
        for lat, lon in mission_waypoints:
            print(f"[DRONE] 좌표 {lat},{lon} 이동")
            time.sleep(1)
            captured = self.lower_od.detect_and_align_fire(timeout=3)
            if captured:
                self.saved_fire_data.append((captured,(lat,lon)))
                print(f"[DRONE] 화재 발견, 로컬 저장: {captured}, 좌표: {lat},{lon}")
        print("[DRONE] 순찰 완료 → 착륙")
        self.landing.run()
        self.send_fire_reports_after_landing()

    def send_fire_report(self, img_path, coords):
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
                s.connect((self.control_center_ip,self.control_center_port))
                with open(img_path,"rb") as f:
                    img_data = f.read()
                header = {"coordinates":{"lat":coords[0],"lon":coords[1]},"image_size":len(img_data)}
                s.sendall(bytes(json.dumps(header).ljust(1024),"utf-8"))
                ack = s.recv(1024)
                if ack==b"HEADER_OK":
                    s.sendall(img_data)
                    print(f"[REPORT] {img_path} 전송 완료")
        except Exception as e:
            print(f"[REPORT ERROR] {e}")

    def send_fire_reports_after_landing(self):
        for img_path, coords in self.saved_fire_data:
            self.send_fire_report(img_path, coords)
        self.saved_fire_data.clear()
        print("[REPORT] 모든 화재 보고 완료")

# ------------------------
# 실행 예시
# ------------------------
if __name__=="__main__":
    pm = PatrolManager()
    waypoints = [(37.5665,126.9780),(37.5666,126.9781)]  # 예시 경로
    pm.patrol_loop(waypoints)
