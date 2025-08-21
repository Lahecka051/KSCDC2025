import socket
import json
import time
import cv2
from Object_Data import Object_Data
from Landing import Landing
from integrated_drone_system import IntegratedDroneSystem, GPSData, ControlMode
import math

class PatrolManager:
    def __init__(self, drone_system: IntegratedDroneSystem, cam_index=0, marker_path="/home/kscdc2025/Marker.png"):
        self.drone_system = drone_system
        self.cam = cv2.VideoCapture(cam_index) # 전방 및 하부 카메라 모두 이 하나의 cam 인덱스로 처리
        self.object_detector = Object_Data(self.cam, mode="upper")
        self.landing = Landing(self.cam, self.drone_system, marker_path)
        self.saved_fire_data = []
        self.control_center_ip = "192.168.0.10"
        self.control_center_port = 4000
        self.current_location = (37.5665, 126.9780)

        self.drone_system.set_gps_callback(self._update_current_location)

    def _update_current_location(self, gps_data: GPSData):
        self.current_location = (gps_data.latitude, gps_data.longitude)
    
    # 화재 지점 GPS 추정 함수
    def _estimate_fire_gps(self, drone_gps: GPSData, angle_x: float, angle_y: float) -> tuple:
        # 이 함수는 드론의 현재 GPS, Yaw, 피치, 롤, 고도, 그리고 카메라 시야각을
        # 기반으로 지상의 화재 지점 GPS를 추정합니다.
        # 실제 구현은 복잡하며, 여기서는 단순화된 가정을 사용합니다.
        
        # 가정: angle_x는 좌우 회전각, angle_y는 상하 기울기각 (피치)
        # GPS 이동은 단순화된 GPS 좌표계에서 계산됩니다.
        earth_radius = 6371000  # 지구 반지름 (미터)
        
        # 각도를 라디안으로 변환
        angle_x_rad = math.radians(angle_x)
        angle_y_rad = math.radians(angle_y)
        heading_rad = math.radians(drone_gps.heading)
        
        # 고도와 각도를 이용해 지상과의 수평 거리 계산
        # 드론 자세(피치, 롤)를 고려해야 하지만, 여기서는 단순화를 위해 생략
        horizontal_distance = drone_gps.altitude * math.tan(angle_y_rad)
        
        # 새로운 GPS 좌표 계산 (간단한 평면 지구 모델 가정)
        delta_lat = (horizontal_distance * math.cos(heading_rad + angle_x_rad)) / earth_radius
        delta_lon = (horizontal_distance * math.sin(heading_rad + angle_x_rad)) / (earth_radius * math.cos(math.radians(drone_gps.latitude)))
        
        estimated_lat = drone_gps.latitude + math.degrees(delta_lat)
        estimated_lon = drone_gps.longitude + math.degrees(delta_lon)

        return (estimated_lat, estimated_lon)

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

    def run(self, mission_waypoints):
        fire_found = False
        fire_gps_estimate = None
        
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
                current_gps = self.drone_system.get_gps()
                if not current_gps:
                    time.sleep(0.1)
                    continue

                ret, frame = self.cam.read()
                if ret:
                    fire_detected, center_coords, _ = self.object_detector.detect_fire_upper()
                    if fire_detected:
                        print(f"[DRONE] 전방 카메라로 화재 감지! 좌표 {self.current_location}에서 재확인 및 GPS 추정 시작.")
                        fire_found = True
                        
                        # 5초간 호버링하며 화재 재확인 및 GPS 추정
                        L1, L2, L3, L4 = "level", "hover", current_gps.heading, 60.0
                        self.drone_system.set_command(L1, L2, L3, L4)
                        
                        is_fire_confirmed = True
                        hover_start_time = time.time()
                        while time.time() - hover_start_time < 5:
                            ret_hover, hover_frame = self.cam.read()
                            if not ret_hover: continue
                            
                            fire_detected_hover, center_coords_hover, _ = self.object_detector.detect_fire_upper()
                            if not fire_detected_hover:
                                is_fire_confirmed = False
                                print("[DRONE] 5초간 화재 재확인 실패. 순찰 재개.")
                                break
                            
                            # 화재 지점의 각도 계산 (가정: 카메라 시야각 90도, 프레임 중심에서 픽셀당 각도 계산)
                            frame_center_x, frame_center_y = self.object_detector.frame_width // 2, self.object_detector.frame_height // 2
                            pixels_per_degree = self.object_detector.frame_width / 90.0
                            angle_x = (center_coords_hover[0] - frame_center_x) / pixels_per_degree
                            angle_y = (center_coords_hover[1] - frame_center_y) / pixels_per_degree
                            
                            # FC의 현재 자세 데이터를 사용하여 GPS 추정
                            current_attitude = self.drone_system.get_attitude() # 가정: 이 함수가 존재하고 Yaw, Pitch, Roll 반환
                            fire_gps_estimate = self._estimate_fire_gps(current_gps, angle_x, angle_y)
                            print(f"[DRONE] 화재 GPS 추정: {fire_gps_estimate}")
                            time.sleep(0.5)

                        break
                time.sleep(0.1)
            
        if fire_found and fire_gps_estimate:
            print(f"[DRONE] 화재 추정 지점({fire_gps_estimate})으로 이동 중...")
            self.drone_system.goto_gps(fire_gps_estimate[0], fire_gps_estimate[1])
            while self.drone_system.mode == ControlMode.GPS:
                self.drone_system.get_gps()
                time.sleep(1)

            print("[DRONE] 화재 지점 상공 도착. 하부 카메라로 전환하여 정렬 시작.")
            # Object_Data의 모드를 'lower'로 변경하고, 하부 카메라용 함수 호출
            self.object_detector.mode = "lower"
            
            # detect_and_align_fire 함수가 드론 제어 명령을 보낼 수 있도록 drone_system.set_command를 전달
            captured_path = self.object_detector.detect_and_align_fire(drone_send_command=self.drone_system.set_command, timeout=20)
            
            if captured_path:
                current_gps_final = self.drone_system.get_gps()
                final_coords = (current_gps_final.latitude, current_gps_final.longitude)
                self.saved_fire_data.append((captured_path, final_coords))
                print(f"[DRONE] 화재 사진 촬영 완료. 최종 좌표: {final_coords}")

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
                print("[DRONE] 하부 카메라 정렬 실패. 순찰 재개.")
        
        print("[DRONE] 모든 순찰 경로 완료 또는 화재 처리 실패. 드론 스테이션으로 복귀.")
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
        pm.run(mission)
    else:
        print("[DRONE] 수신된 웨이포인트가 없습니다. 임무를 시작할 수 없습니다.")
