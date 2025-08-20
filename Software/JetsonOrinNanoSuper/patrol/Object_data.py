from ultralytics import YOLO
import cv2
import numpy as np
from pipeline import gstreamer_pipeline

class Object_Data:
    def __init__(self, cap):
        self.model = YOLO("best.engine")
        self.cap = cap

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

        if (vertical, horizontal) == ("위", "왼쪽"): return ["level", "forward_left", 0, 10]
        if (vertical, horizontal) == ("위", "가운데"): return ["level", "forward", 0, 10]
        if (vertical, horizontal) == ("위", "오른쪽"): return ["level", "forward_right", 0, 10]
        if (vertical, horizontal) == ("가운데", "왼쪽"): return ["level", "left", 0, 10]
        if (vertical, horizontal) == ("가운데", "가운데"): return ["level", "stop", 0, 10]
        if (vertical, horizontal) == ("가운데", "오른쪽"): return ["level", "right", 0, 10]
        if (vertical, horizontal) == ("아래", "왼쪽"): return ["level", "backward_left", 0, 10]
        if (vertical, horizontal) == ("아래", "가운데"): return ["level", "backward", 0, 10]
        if (vertical, horizontal) == ("아래", "오른쪽"): return ["level", "backward_right", 0, 10]
        return None

    def detect_fire(self):
        """PatrolManager에서 호출할 단일 프레임 화재 탐지"""
        ret, frame = self.cap.read()
        if not ret:
            print("프레임 읽기 실패")
            return False, None, None

        results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)

        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            x1, y1, x2, y2 = best_box.xyxy[0]
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            if class_name.lower() in ["fire", "flame"]:
                image_path = "detected_fire.jpg"
                cv2.imwrite(image_path, frame)
                command = self.get_position_command(center_x, center_y)
                print("화재 탐지됨, 이미지 저장 완료, 방향:", command)
                return True, image_path, command

        return False, None, None

    def run_OD(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("카메라 프레임 읽기 실패")
                break

            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
            annotated_frame = results[0].plot()

            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda box: box.conf[0])
                class_name = self.class_names[int(best_box.cls[0])]
                x1, y1, x2, y2 = best_box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                command = self.get_position_command(center_x, center_y)
                if command:
                    print(f"→ {class_name} (conf: {best_box.conf[0]:.2f}): {command}")

            cv2.imshow("YOLOv8 Detection (Optimized)", annotated_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()
