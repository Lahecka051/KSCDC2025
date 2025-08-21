import cv2
import numpy as np
import time
from integrated_drone_system import IntegratedDroneSystem, GPSData

class Landing:
    def __init__(self, cap, drone_system: IntegratedDroneSystem, marker_path="/home/kscdc2025/Marker.png"):
        self.cap = cap
        self.drone_system = drone_system
        self.marker_path = marker_path
        self.marker_color = cv2.imread(marker_path)
        if self.marker_color is None:
            print(f"Error: Marker not found at {marker_path}")
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.step_x = self.frame_width // 3
        self.step_y = self.frame_height // 3
        self.last_print_time = 0
        self.station_altitude = 1.0 # 착륙 스테이션의 초기 고도 (미터)

    def detect_red_dot(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0,150,150])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([160,150,150])
        upper_red2 = np.array([179,255,255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((3,3),np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE,kernel)
        contours,_ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours,key=cv2.contourArea)
            if cv2.contourArea(largest) > 150:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
        return None

    def process_frame(self, frame):
        marker_center = self.detect_red_dot(frame)
        cmd = None
        if marker_center:
            pos_x = ""
            if marker_center[0] < self.frame_width / 3:
                pos_x = "left"
            elif marker_center[0] > 2 * self.frame_width / 3:
                pos_x = "right"
            else:
                pos_x = "center"

            pos_y = ""
            if marker_center[1] < self.frame_height / 3:
                pos_y = "top"
            elif marker_center[1] > 2 * self.frame_height / 3:
                pos_y = "bottom"
            else:
                pos_y = "center"
            
            if pos_x == "center" and pos_y == "center":
                cmd = ["level", "hover", 0, 60]
            elif pos_x == "left" and pos_y == "top":
                cmd = ["level", "forward_left", 0, 20]
            elif pos_x == "center" and pos_y == "top":
                cmd = ["level", "forward", 0, 20]
            elif pos_x == "right" and pos_y == "top":
                cmd = ["level", "forward_right", 0, 20]
            elif pos_x == "left" and pos_y == "center":
                cmd = ["level", "left", 0, 20]
            elif pos_x == "right" and pos_y == "center":
                cmd = ["level", "right", 0, 20]
            elif pos_x == "left" and pos_y == "bottom":
                cmd = ["level", "backward_left", 0, 20]
            elif pos_x == "center" and pos_y == "bottom":
                cmd = ["level", "backward", 0, 20]
            elif pos_x == "right" and pos_y == "bottom":
                cmd = ["level", "backward_right", 0, 20]

        debug_frame = frame.copy()
        if marker_center:
            cv2.circle(debug_frame, marker_center, 10, (0, 255, 0), -1)
        
        cv2.line(debug_frame, (self.step_x, 0), (self.step_x, self.frame_height), (255, 0, 0), 2)
        cv2.line(debug_frame, (2 * self.step_x, 0), (2 * self.step_x, self.frame_height), (255, 0, 0), 2)
        cv2.line(debug_frame, (0, self.step_y), (self.frame_width, self.step_y), (255, 0, 0), 2)
        cv2.line(debug_frame, (0, 2 * self.step_y), (self.frame_width, 2 * self.step_y), (255, 0, 0), 2)

        return cmd, marker_center, debug_frame

    def run(self):
        aligned_count = 0
        print("[LANDING] 착륙 마커 감지 및 정렬 시작...")

        while True:
            ret, frame = self.cap.read()
            if not ret: 
                print("[LANDING ERROR] 카메라 프레임 읽기 실패. 착륙 중단.")
                break
            
            cmd, marker_center, debug_frame = self.process_frame(frame)
            
            if cmd:
                L1, L2, L3, L4 = cmd[0], cmd[1], cmd[2], float(cmd[3])
                self.drone_system.set_command(L1, L2, L3, L4)
                
                if L2 == "hover":
                    aligned_count += 1
                    print(f"[LANDING] 마커 중앙 정렬 중... ({aligned_count}/6)")
                else:
                    aligned_count = 0
            else:
                aligned_count = 0
                print("[LANDING] 마커를 찾지 못함. 호버링 중...")
                L1, L2, L3, L4 = "level", "hover", 0, 60.0
                self.drone_system.set_command(L1, L2, L3, L4)
            
            if aligned_count >= 6:
                print("[LANDING] 중앙 정렬 완료. 착륙 시작!")
                L1, L2, L3, L4 = "down", "hover", 0, 50.0
                self.drone_system.set_command(L1, L2, L3, L4)
                
                # 착륙 고도 판별
                while True:
                    current_gps = self.drone_system.get_gps()
                    if current_gps and current_gps.altitude <= self.station_altitude + 0.1: # 고도가 착륙 고도에 근접하면
                        print(f"[LANDING] 착륙 완료! 최종 고도: {current_gps.altitude:.2f}m")
                        L1, L2, L3, L4 = "level", "hover", 0, 0.0
                        self.drone_system.set_command(L1, L2, L3, L4)
                        break
                    print(f"[LANDING] 하강 중... 현재 고도: {current_gps.altitude:.2f}m")
                    time.sleep(1)

                break
            
            now = time.time()
            if now - self.last_print_time > 0.5:
                self.last_print_time = now
                cv2.imshow("Landing Detection", debug_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                print("[LANDING] 디버그 모드 종료 요청. 착륙 중단.")
                break
