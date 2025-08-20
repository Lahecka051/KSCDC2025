# PatrolManager 개선 (하부 카메라 Object_Data 호출)
import time
import cv2
import os
import json
import socket
from Object_Data import Object_Data
from Landing import Landing

class PatrolManager:
    def __init__(self, upper_cam_index=0, lower_cam_index=1, marker_path="/home/kscdc2025/Marker.png"):
        # 전방 카메라: 화재 탐지
        self.upper_cam = cv2.VideoCapture(upper_cam_index)
        self.upper_od = Object_Data(self.upper_cam, mode="upper")
        
        # 하부 카메라: 중앙 정렬 + 촬영
        self.lower_cam = cv2.VideoCapture(lower_cam_index)
        self.lower_od = Object_Data(self.lower_cam, mode="lower")
        
        # 착륙 클래스
        self.landing = Landing(self.lower_cam, marker_path=marker_path)
        
        # 상태 변수
        self.fire_coordinates = None
        self.fire_image_path = None
        self.control_center_ip = "192.168.0.10"
        self.control_center_port = 4000

    def patrol_loop(self):
        print("순찰 시작")
        while True:
            # --- 전방 화재 탐지 ---
            detected, coords, frame = self.upper_od.detect_fire_upper()
            if detected:
                print("전방 화재 감지 → 5초 호버링 중 확인...")
                self.fire_coordinates = coords
                hover_start = time.time()
                fire_confirmed = False
                
                # 5초 동안 재확인
                while time.time() - hover_start < 5:
                    detected_hover, coords_hover, _ = self.upper_od.detect_fire_upper()
                    if detected_hover:
                        fire_confirmed = True
                    time.sleep(0.1)
                
                if fire_confirmed:
                    print("화재 확정 → 하부 카메라 중앙 정렬 및 촬영")
                    # Object_Data 클래스 하부 카메라 메서드 호출
                    captured_path = self.lower_od.detect_and_align_fire(timeout=5)
                    if captured_path:
                        self.fire_image_path = captured_path
                        print(f"사진 촬영 완료: {self.fire_image_path}")
                    else:
                        print("하부 카메라 사진 촬영 실패 또는 타임아웃")

                    print("착륙 시작")
                    self.landing.run()

                    print("관제 센터로 화재 보고 전송")
                    self.send_fire_report()
                else:
                    print("호버링 중 화재 미확인 → 순찰 계속")

            time.sleep(0.1)

    def send_fire_report(self):
        """착륙 후 화재 이미지 관제 센터 전송"""
        if not self.fire_image_path:
            return
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.control_center_ip, self.control_center_port))
                with open(self.fire_image_path, "rb") as f:
                    img_data = f.read()
                header = {"coordinates": self.fire_coordinates, "image_size": len(img_data)}
                s.sendall(bytes(json.dumps(header).ljust(1024), "utf-8"))
                
                ack = s.recv(1024)
                if ack == b"HEADER_OK":
                    s.sendall(img_data)
                    print("화재 이미지 전송 완료!")
        except Exception as e:
            print(f"화재 이미지 전송 오류: {e}")


if __name__ == "__main__":
    patrol_manager = PatrolManager(marker_path="/home/kscdc2025/Marker.png")
    patrol_manager.patrol_loop()
