# capture_on_detect.py
import cv2
import time
from ultralytics import YOLO
from pipeline import gstreamer_pipeline

class Object_Capture:
    def __init__(self, cap, target_class, save_dir="captures"):
        self.model = YOLO("best.engine")
        self.cap = cap
        self.target_class = target_class
        self.save_dir = save_dir

        import os
        os.makedirs(save_dir, exist_ok=True)

        self.class_names = self.model.names
        self.detect_start_time = None  # 객체가 처음 감지된 시각
        self.detecting = False         # 현재 감지 중인지 여부

    def run_and_capture(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("카메라 프레임 읽기 실패")
                break

            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
            annotated_frame = results[0].plot()

            detected = False
            for box in results[0].boxes:
                class_name = self.class_names[int(box.cls[0])]
                if class_name == self.target_class:
                    detected = True
                    break

            if detected:
                if not self.detecting:
                    self.detecting = True
                    self.detect_start_time = time.time()
                else:
                    elapsed = time.time() - self.detect_start_time
                    if elapsed >= 5:
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        filename = f"{self.save_dir}/{self.target_class}_{timestamp}.jpg"
                        cv2.imwrite(filename, frame)
                        print(f"[저장 완료] {filename} (5초 연속 감지)")
                        self.detecting = False  # 다음 감지를 위해 리셋
            else:
                self.detecting = False
                self.detect_start_time = None

            cv2.imshow("YOLOv8 Detection + Capture", annotated_frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    pipeline = gstreamer_pipeline(sensor_id=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("GStreamer 파이프라인 열기 실패")
    else:
        obj_cap = Object_Capture(cap, target_class="fire")
        obj_cap.run_and_capture()

