import cv2
import numpy as np
import time
from ultralytics import YOLO

class Object_Data:
    def __init__(self, cap, mode="upper"):
        self.cap = cap
        self.mode = mode
        self.model = YOLO("best.engine")
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
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
        # 상부 카메라 모드에서만 동작
        if self.mode != "upper": return False, None, None
        ret, frame = self.cap.read()
        if not ret: return False, None, None
        results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            if class_name.lower() in ["fire", "smoke"]:
                x1,y1,x2,y2 = best_box.xyxy[0]
                center_x = int((x1+x2)/2)
                center_y = int((y1+y2)/2)
                # print(f"[UPPER FIRE] {class_name} detected at {center_x},{center_y}")
                return True, (center_x, center_y), frame
        return False, None, None

    def detect_and_align_fire(self, drone_send_command: callable, timeout=20):
        # 하부 카메라 모드에서만 동작
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
                if class_name.lower() in ["fire", "smoke"]:
                    x1,y1,x2,y2 = best_box.xyxy[0]
                    cx = int((x1+x2)/2)
                    cy = int((y1+y2)/2)
                    cmd = self.get_position_command(cx,cy)
                    if cmd:
                        print(f"[LOWER ALIGN] {class_name} at {cx},{cy}, cmd: {cmd}")
                        # 드론 제어 명령 전송
                        L1, L2, L3, L4 = cmd[0], cmd[1], cmd[2], float(cmd[3])
                        drone_send_command(L1, L2, L3, L4)
                        
                        if L2 == "stop":
                            captured_path = f"lower_fire_{int(time.time())}.jpg"
                            cv2.imwrite(captured_path, frame)
                            print("[LOWER FIRE] 하부 카메라로 촬영 완료.")
                            return captured_path
            
            # 5초 간격으로 화재가 감지되지 않으면 종료 (안전장치)
            if time.time()-start > 5 and not captured_path:
                print("[LOWER ALIGN] 5초간 화재 감지 실패. 정렬 중단.")
                return None
        return captured_path
