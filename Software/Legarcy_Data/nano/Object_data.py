# Object_data.py

from ultralytics import YOLO
import cv2
import numpy as np
from pipeline import gstreamer_pipeline

class Object_Data:
    def __init__(self, cap):
        self.model = YOLO("best.engine")
        self.cap = cap

        # [수정] GStreamer 파이프라인에서 해상도가 고정되므로 불필요한 cap.set 제거
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 실제 입력되는 프레임 크기(960x540)에 맞춰 3x3 그리드 계산
        self.w1, self.w2 = self.frame_width // 3, 2 * self.frame_width // 3
        self.h1, self.h2 = self.frame_height // 3, 2 * self.frame_height // 3

        self.class_names = self.model.names

    def get_position_command(self, x, y):
        # [개선] 함수가 객체 상태를 직접 바꾸지 않고, 계산된 결과만 반환하도록 변경
        if y < self.h1: vertical = "위"
        elif y < self.h2: vertical = "가운데"
        else: vertical = "아래"

        if x < self.w1: horizontal = "왼쪽"
        elif x < self.w2: horizontal = "가운데"
        else: horizontal = "오른쪽"

        # 각 위치에 맞는 명령어 리스트를 반환
        if (vertical, horizontal) == ("위", "왼쪽"): return ["level", "forward_left", 0, 10]
        if (vertical, horizontal) == ("위", "가운데"): return ["level", "forward", 0, 10]
        if (vertical, horizontal) == ("위", "오른쪽"): return ["level", "forward_right", 0, 10]
        if (vertical, horizontal) == ("가운데", "왼쪽"): return ["level", "left", 0, 10]
        if (vertical, horizontal) == ("가운데", "가운데"): return ["level", "stop", 0, 10]
        if (vertical, horizontal) == ("가운데", "오른쪽"): return ["level", "right", 0, 10]
        if (vertical, horizontal) == ("아래", "왼쪽"): return ["level", "backward_left", 0, 10]
        if (vertical, horizontal) == ("아래", "가운데"): return ["level", "backward", 0, 10]
        # [수정] 명령어 오타 수정
        if (vertical, horizontal) == ("아래", "오른쪽"): return ["level", "backward_right", 0, 10]

        return None # 해당하는 위치가 없을 경우 None 반환
    def run_OD(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("카메라 프레임을 읽어오는 데 실패했습니다.")
                break

            # [성능 개선] 불필요한 프레임 확대를 제거하고 원본 프레임(960x540)을 바로 사용
            # imgsz는 모델 학습 시 사용된 크기나, 성능에 맞춰 640 등으로 조절하는 것을 권장
            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
            annotated_frame = results[0].plot()

            # [수정] 여러 객체 탐지 시, 가장 신뢰도(confidence)가 높은 객체 하나만 처리
            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda box: box.conf[0])

                class_name = self.class_names[int(best_box.cls[0])]

                # 중심 좌표 계산
                x1, y1, x2, y2 = best_box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                command = self.get_position_command(center_x, center_y)
                if command:
                    print(f"→ {class_name} (conf: {best_box.conf[0]:.2f}): {command}")

            # 결과 출력
            cv2.imshow("YOLOv8 Detection (Optimized)", annotated_frame)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    pipeline0 = gstreamer_pipeline(sensor_id=0)
    pipeline1 = gstreamer_pipeline(sensor_id=1)
    cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
    cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)
    if not(cap0.isOpened() and  cap1.isOpened()):
        print("GStreamer 파이프라인을 열 수 없습니다. 카메라 연결이나 설정을 확인하세요.")
    else:
        ob0 = Object_Data(cap0)
        ob1 = Object_Data(cap1)
        ob0.run_OD()
        ob1.run_OD()

