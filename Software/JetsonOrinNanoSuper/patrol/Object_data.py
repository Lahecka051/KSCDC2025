from ultralytics import YOLO
import cv2
import numpy as np
import time
from pipeline import gstreamer_pipeline

class Object_Data:
    def __init__(self, cap, mode="upper"):
        self.model = YOLO("best.engine")
        self.cap = cap
        self.mode = mode  # "upper" or "lower"

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 3x3 그리드 계산
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
            ("위","왼쪽"): ["level", "forward_left", 0, 10],
            ("위","가운데"): ["level", "forward", 0, 10],
            ("위","오른쪽"): ["level", "forward_right", 0, 10],
            ("가운데","왼쪽"): ["level", "left", 0, 10],
            ("가운데","가운데"): ["level", "stop", 0, 10],
            ("가운데","오른쪽"): ["level", "right", 0, 10],
            ("아래","왼쪽"): ["level", "backward_left", 0, 10],
            ("아래","가운데"): ["level", "backward", 0, 10],
            ("아래","오른쪽"): ["level", "backward_right", 0, 10]
        }
        return mapping.get((vertical, horizontal), None)

    # ------------------------
    # 전방 카메라 단일 프레임 탐지
    # ------------------------
    def detect_fire_upper(self):
        if self.mode != "upper":
            raise ValueError("이 메서드는 전방 카메라용입니다 (mode='upper').")
        ret, frame = self.cap.read()
        if not ret:
            return False, None, None

        results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            x1, y1, x2, y2 = best_box.xyxy[0]
            center_x = int((x1 + x2)/2)
            center_y = int((y1 + y2)/2)
            if class_name.lower() in ["fire", "flame"]:
                image_path = "upper_detected_fire.jpg"
                cv2.imwrite(image_path, frame)
                command = self.get_position_command(center_x, center_y)
                print(f"전방 화재 탐지 → {class_name}, 중앙 좌표: {center_x},{center_y}, 명령: {command}")
                return True, (center_x, center_y), frame
        return False, None, None

    # ------------------------
    # 하부 카메라 중앙 정렬 후 촬영
    # ------------------------
    def detect_and_align_fire(self, drone_send_command=None, timeout=5):
        if self.mode != "lower":
            raise ValueError("이 메서드는 하부 카메라용입니다 (mode='lower').")
        start_time = time.time()
        captured_path = None
        while time.time() - start_time < timeout:
            ret, frame = self.cap.read()
            if not ret:
                continue
            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda box: box.conf[0])
                class_name = self.class_names[int(best_box.cls[0])]
                x1, y1, x2, y2 = best_box.xyxy[0]
                center_x = int((x1 + x2)/2)
                center_y = int((y1 + y2)/2)
                command = self.get_position_command(center_x, center_y)
                if command:
                    print(f"하부 카메라 정렬 → {class_name} 위치: {command}")
                    if drone_send_command:
                        drone_send_command(command)
                    if command[1] == "stop":
                        captured_path = "lower_captured_fire.jpg"
                        cv2.imwrite(captured_path, frame)
                        break
        return captured_path

    # ------------------------
    # 기존 OD 루프
    # ------------------------
    def run_OD(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
            annotated_frame = results[0].plot()
            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda box: box.conf[0])
                class_name = self.class_names[int(best_box.cls[0])]
                x1, y1, x2, y2 = best_box.xyxy[0]
                center_x = int((x1 + x2)/2)
                center_y = int((y1 + y2)/2)
                command = self.get_position_command(center_x, center_y)
                if command:
                    print(f"→ {class_name} (conf: {best_box.conf[0]:.2f}): {command}")
            cv2.imshow("YOLOv8 Detection", annotated_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        self.cap.release()
        cv2.destroyAllWindows()
