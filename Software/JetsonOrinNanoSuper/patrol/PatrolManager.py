import socket
import json
import time
import cv2
from Object_Data import Object_Data
from Landing import Landing
from integrated_drone_system import IntegratedDroneSystem, GPSData, ControlMode

class PatrolManager:
    def __init__(self, drone_system: IntegratedDroneSystem, cam_index=0, marker_path="/home/kscdc2025/Marker.png"):
        self.drone_system = drone_system
        self.cam = cv2.VideoCapture(cam_index)
        self.object_detector = Object_Data(self.cam)
        self.landing = Landing(self.cam, self.drone_system, marker_path)
        self.saved_fire_data = []
        self.control_center_ip = "192.168.0.10"
        self.control_center_port = 4000
        self.current_location = (37.5665, 126.9780)

        self.drone_system.set_gps_callback(self._update_current_location)

    def _update_current_location(self, gps_data: GPSData):
        self.current_location = (gps_data.latitude, gps_data.longitude)

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

    def run_mission(self, mission_waypoints):
        fire_found = False
        current_yaw = 0.0
        
        if not self.drone_system.is_connected:
            if not self.drone_system.connect():
                print("[DRONE] FC 연결 실패! 임무를 시작할 수 없습니다.")
                return

        print("[DRONE] 이륙 중...")
        L1, L2, L3, L4 = "up", "forward", 0.0, 90.0
        self.drone_system.set_command(L1, L2, L3, L4)
        time.sleep(5) 
        L1, L2, L3, L4 = "level", "hover", 0.0, 60.0
        self.drone_system.set_command(L1, L2, L3, L4)
        time.sleep(2)
        
        self.drone_system.set_home() 

        for lat, lon in mission_waypoints:
            if fire_found:
                break
            
            print(f"[DRONE] 다음 좌표로 이동: {lat},{lon}")
            self.drone_system.goto_gps(lat, lon)
            
            while self.drone_system.mode == ControlMode.GPS:
                self.drone_system.get_gps() 
                
                ret, frame = self.cam.read()
                if ret:
                    fire_detected, center_coords, _ = self.object_detector.detect_fire_upper()
                    if fire_detected:
                        print(f"[DRONE] 화재 감지! 좌표 {self.current_location}에서 정렬 시작.")
                        fire_found = True
                        attitude = self.drone_system.get_attitude()
                        if attitude:
                            current_yaw = attitude.yaw
                            print(f"[DRONE] 현재 Yaw 각도: {current_yaw:.2f}도. 각도 유지하며 호버링.")
                        
                        L1, L2, L3, L4 = "level", "hover", current_yaw, 60.0
                        self.drone_system.set_command(L1, L2, L3, L4) 
                        time.sleep(1) 
                        break
                time.sleep(0.1)
            
            if fire_found:
                print("[DRONE] 화재를 상단 카메라 중앙에 정렬 중...")
                start_time = time.time()
                aligned_count = 0
                while aligned_count < 5 and time.time() - start_time < 10:
                    ret, align_frame = self.cam.read()
                    if not ret: continue
                    
                    fire_detected, center_coords, _ = self.object_detector.detect_fire_upper()
                    
                    if fire_detected:
                        cmd = self.object_detector.get_position_command(center_coords[0], center_coords[1])
                        if cmd:
                            L1, L2, L3, L4 = cmd[0], cmd[1], current_yaw, float(cmd[3])
                            self.drone_system.set_command(L1, L2, L3, L4)
                            
                            if L2 == "stop":
                                aligned_count += 1
                                print(f"[DRONE] 화재 중앙 정렬 중... ({aligned_count}/5)")
                            else:
                                aligned_count = 0
                        else:
                            aligned_count = 0
                    else:
                        print("[DRONE] 정렬 중 화재 소실. 순찰 재개.")
                        fire_found = False
                        L1, L2, L3, L4 = "level", "hover", 0.0, 60.0
                        self.drone_system.set_command(L1, L2, L3, L4)
                        time.sleep(1)
                        break
                    
                    time.sleep(0.5)
                
                if aligned_count >= 5:
                    print("[DRONE] 5초간 호버링하며 화재 판별...")
                    is_fire_confirmed = True
                    hover_start_time = time.time()
                    while time.time() - hover_start_time < 5:
                        ret, hover_frame = self.cam.read()
                        if not ret: continue
                        fire_detected, _, _ = self.object_detector.detect_fire_upper()
                        if not fire_detected:
                            is_fire_confirmed = False
                            break
                        time.sleep(0.5)
                    
                    if is_fire_confirmed:
                        captured_path = f"fire_report_{int(time.time())}.jpg"
                        cv2.imwrite(captured_path, hover_frame) 
                        self.saved_fire_data.append((captured_path, self.current_location))
                        print(f"[DRONE] 화재 확정! 사진 저장 완료: {captured_path}, 좌표: {self.current_location}")
                        
                        print("[DRONE] 즉시 드론 스테이션으로 복귀 명령 발송.")
                        self.drone_system.return_home()
                        
                        while self.drone_system.mode == ControlMode.GPS:
                            self.drone_system.get_gps()
                            time.sleep(1)
                            
                        print("[DRONE] 드론 스테이션 도착, 착륙 시작.")
                        self.landing.run()
                        
                        print("[DRONE] 착륙 완료, Wi-Fi 연결. 보고서 전송.")
                        self.send_fire_reports_after_landing()
                        return
                    else:
                        print("[DRONE] 5초간 화재 재확인 실패. 순찰 재개.")
                        fire_found = False
                        L1, L2, L3, L4 = "level", "hover", 0.0, 60.0
                        self.drone_system.set_command(L1, L2, L3, L4)
                        time.sleep(1)
            
        if not fire_found:
            print("[DRONE] 모든 순찰 경로 완료. 드론 스테이션으로 복귀.")
            self.drone_system.return_home()
            
            while self.drone_system.mode == ControlMode.GPS:
                self.drone_system.get_gps()
                time.sleep(1)

            print("[DRONE] 드론 스테이션 도착, 착륙 시작.")
            self.landing.run()
            print("[DRONE] 착륙 완료.")
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
        self.drone_system.disconnect()

if __name__ == "__main__":
    drone_controller = IntegratedDroneSystem(port="/dev/ttyTHS1", baudrate=115200) 
    pm = PatrolManager(drone_controller) 
    mission = pm.receive_waypoints()
    if mission:
        pm.run_mission(mission)
    else:
        print("[DRONE] 수신된 웨이포인트가 없습니다. 임무를 시작할 수 없습니다.")
