import cv2
import numpy as np
import time

class Landing:
    def __init__(self, cap, marker_path="/home/kscdc2025/Marker.png"):
        self.cap = cap
        self.marker_path = marker_path
        self.marker_color = cv2.imread(marker_path)
        if self.marker_color is None:
            raise FileNotFoundError(f"Marker not found: {marker_path}")
        self.frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.step_x = self.frame_width // 3
        self.step_y = self.frame_height // 3
        self.last_print_time = 0

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
            col = marker_center[0]//self.step_x +1
            row = marker_center[1]//self.step_y +1
            pos = (row-1)*3 + col
            outer_cmds = {
                1:["level","forward_left",0,10],2:["level","forward",0,10],3:["level","forward_right",0,10],
                4:["level","left",0,10],6:["level","right",0,10],
                7:["level","backward_left",0,10],8:["level","backward",0,10],9:["level","backward_right",0,10]
            }
            if pos in outer_cmds: cmd = outer_cmds[pos]
            elif pos==5: cmd=["level","stop",0,5]
        return cmd, marker_center, frame

    def send_drone_command(self, cmd):
        if cmd: print(f"[DRONE CMD] {cmd}")

    def run(self):
        aligned_count = 0
        while True:
            ret, frame = self.cap.read()
            if not ret: break
            cmd, marker_center, debug_frame = self.process_frame(frame)
            self.send_drone_command(cmd)
            if cmd and cmd[1]=="stop": aligned_count+=1
            else: aligned_count=0
            if aligned_count>=6:
                print("[LANDING] 중앙 정렬 완료. 착륙 시작!")
                self.send_drone_command(["land"])
                break
            now = time.time()
            if now - self.last_print_time>0.5:
                self.last_print_time = now
                cv2.imshow("Landing Detection", debug_frame)
            if cv2.waitKey(1) & 0xFF==27: break
        self.cap.release()
        cv2.destroyAllWindows()
