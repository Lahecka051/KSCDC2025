from ultralytics import YOLO
import cv2
import time
import numpy as np
from pipeline import gstreamer_pipeline

class Object_Data:
    def __init__(self, cap, mode="upper"):
        self.cap = cap
        self.mode = mode  # "upper" or "lower"
        self.model = YOLO("best.engine")
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 3x3 그리드
        self.w1, self.w2 = self.frame_width // 3, 2 * self.frame_width // 3
        self.h1, self.h2 = self.frame_height // 3, 2 * self.frame_height // 3
        self.class_names = self.model.names

    def get_position_command(self, x, y):
        if y < self.h1: vertical = "위"
        elif y < self.h2: vertical = "가운데"
        else: vertical = "아래"
        if x < self.w1: horizontal = "왼쪽"
        elif x < self.w2: horizontal = "가운데"
        else: horizontal = "오른쪽"
        mapping = {
            ("위","왼쪽"): ["level","forward_left",0,10],
            ("위","가운데"): ["level","forward",0,10],
            ("위","오른쪽"): ["level","forward_right",0,10],
            ("가운데","왼쪽"): ["level","left",0,10],
            ("가운데","가운데"): ["level","stop",0,10],
            ("가운데","오른쪽"): ["level","right",0,10],
            ("아래","왼쪽"): ["level","backward_left",0,10],
            ("아래","가운데"): ["level","backward",0,10],
            ("아래","오른쪽"): ["level","backward_right",0,10]
        }
        return mapping.get((vertical,horizontal),None)

    def detect_fire_upper(self):
        if self.mode != "upper": return False, None, None
        ret, frame = self.cap.read()
        if not ret: return False, None, None
        results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            x1,y1,x2,y2 = best_box.xyxy[0]
            center_x = int((x1+x2)/2)
            center_y = int((y1+y2)/2)
            if class_name.lower() in ["fire","flame"]:
                image_path = "upper_detected_fire.jpg"
                cv2.imwrite(image_path, frame)
                cmd = self.get_position_command(center_x, center_y)
                print(f"[UPPER FIRE] {class_name} detected at {center_x},{center_y}, cmd: {cmd}")
                return True, (center_x, center_y), frame
        return False, None, None

    def detect_and_align_fire(self, drone_send_command=None, timeout=5):
        if self.mode != "lower": return None
        start = time.time()
        captured_path = None
        while time.time()-start < timeout:
            ret, frame = self.cap.read()
            if not ret: continue
            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda box: box.conf[0])
                class_name = self.class_names[int(best_box.cls[0])]
                x1,y1,x2,y2 = best_box.xyxy[0]
                cx = int((x1+x2)/2)
                cy = int((y1+y2)/2)
                cmd = self.get_position_command(cx,cy)
                if cmd:
                    print(f"[LOWER ALIGN] {class_name} at {cx},{cy}, cmd: {cmd}")
                    if drone_send_command:
                        drone_send_command(cmd)
                    if cmd[1] == "stop":
                        captured_path = f"lower_fire_{int(time.time())}.jpg"
                        cv2.imwrite(captured_path, frame)
                        break
        return captured_path
