import socket
import json
import time
import cv2
from Object_Data import Object_Data
from Landing import Landing

class PatrolManager:
    def __init__(self, cam_index=0, marker_path="/home/kscdc2025/Marker.png"):
        self.cam = cv2.VideoCapture(cam_index)
        self.object_detector = Object_Data(self.cam)
        self.landing = Landing(self.cam, marker_path)
        self.saved_fire_data = []
        self.control_center_ip = "192.168.0.10"
        self.control_center_port = 4000
        self.current_location = (37.5665, 126.9780) # 초기 드론 위치(임의 설정 값)

    def receive_waypoints(self):
        waypoints = []
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(("0.0.0.0", 5000))
            server.listen(1)
            print("[DRONE] 관제 센터 연결 대기중...")
            conn, addr = server.accept()
            with conn:
                print(f"[DRONE] 관제 센터 연결됨: {addr}")
                data = conn.recv(4096).decode("utf-8")
                raw_data = json.loads(data)
                if isinstance(raw_data, list):
                    waypoints = [(item["lat"], item["lon"]) for item in raw_data]
        return waypoints

    def _simulate_drone_command(self, cmd):
        """드론에 실제 명령을 보내는 함수 (테스트용)"""
        print(f"[DRONE CMD] {cmd}")
        time.sleep(0.5)

    def run_mission(self, mission_waypoints):
        fire_found = False
        
        for lat, lon in mission_waypoints:
            if fire_found:
                break
                
            print(f"[DRONE] 다음 좌표로 이동 중: {lat},{lon}")
            # 실제 드론 이동 명령 대체 (이동 중 감지)
            time.sleep(3) 
            self.current_location = (lat, lon)

            # 이동 중 실시간으로 화재 감시
            ret, frame = self.cam.read()
            if ret:
                fire_detected, center_coords, debug_frame = self.object_detector.detect_fire_upper()
                if fire_detected:
                    print(f"[DRONE] 화재 감지! 좌표 {self.current_location}에서 정렬 시작.")
                    fire_found = True

                    # 1. 화재를 상단 카메라 중앙에 정렬
                    start_time = time.time()
                    aligned = False
                    while time.time() - start_time < 10: # 최대 10초간 정렬 시도
                        ret, align_frame = self.cam.read()
                        if not ret: continue
                        
                        fire_detected, center_coords, _ = self.object_detector.detect_fire_upper()
                        if fire_detected:
                            cmd = self.object_detector.get_position_command(center_coords[0], center_coords[1])
                            self._simulate_drone_command(cmd)
                            if cmd[1] == "stop":
                                print("[DRONE] 화재 중앙 정렬 완료!")
                                aligned = True
                                break
                    
                    if aligned:
                        # 2. 5초간 호버링하며 화재 재판별
                        print("[DRONE] 5초간 호버링하며 화재 판별...")
                        is_fire_confirmed = True
                        for _ in range(5):
                            ret, hover_frame = self.cam.read()
                            if not ret: continue
                            fire_detected, _, _ = self.object_detector.detect_fire_upper()
                            if not fire_detected:
                                is_fire_confirmed = False
                                break
                            time.sleep(1)
                        
                        if is_fire_confirmed:
                            # 3. 화재 확정 시 사진 촬영 및 저장
                            captured_path = f"fire_report_{int(time.time())}.jpg"
                            cv2.imwrite(captured_path, hover_frame)
                            self.saved_fire_data.append((captured_path, self.current_location))
                            print(f"[DRONE] 화재 확정! 사진 저장 완료: {captured_path}")
                            print("[DRONE] 즉시 드론 스테이션으로 복귀 명령 발송.")
                            self._simulate_drone_command(["return_to_home"])
                            
                            # 스테이션 복귀 및 보고서 전송 시뮬레이션
                            time.sleep(5) 
                            self.landing.run()
                            print("[DRONE] 드론 스테이션 도착, Wi-Fi 연결.")
                            self.send_fire_reports_after_landing()
                            return # 임무 종료
                        else:
                            print("[DRONE] 5초간 화재 재확인 실패. 순찰 재개.")
                            fire_found = False # 순찰 재개

        if not fire_found:
            print("[DRONE] 모든 순찰 경로 완료. 드론 스테이션으로 복귀.")
            self._simulate_drone_command(["return_to_home"])
            time.sleep(5)
            self.landing.run()
            self.send_fire_reports_after_landing()

    def send_fire_report(self, img_path, coords):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.control_center_ip, self.control_center_port))
                with open(img_path, "rb") as f:
                    img_data = f.read()
                header = {
                    "coordinates": {"lat": coords[0], "lon": coords[1]},
                    "image_size": len(img_data)
                }
                s.sendall(bytes(json.dumps(header).ljust(1024), "utf-8"))
                ack = s.recv(1024)
                if ack == b"HEADER_OK":
                    s.sendall(img_data)
                    print(f"[REPORT] {img_path} 전송 완료")
        except Exception as e:
            print(f"[REPORT ERROR] {e}")

    def send_fire_reports_after_landing(self):
        if not self.saved_fire_data:
            print("[REPORT] 전송할 화재 보고서가 없습니다.")
            return
        
        for img_path, coords in self.saved_fire_data:
            self.send_fire_report(img_path, coords)
        self.saved_fire_data.clear()
        print("[REPORT] 모든 화재 보고 완료")

    def __del__(self):
        if self.cam.isOpened():
            self.cam.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    pm = PatrolManager()
    mission = pm.receive_waypoints()
    if mission:
        pm.run_mission(mission)
