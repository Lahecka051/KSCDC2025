import cv2
import numpy as np

class Landing:
    def __init__(self, marker_path = "Marker.png"):
        self.marker_path = marker_path
        self.marker_color = cv2.imread(self.marker_path)
        if self.marker_color is None:
            raise FileNotFoundError(f"마커 이미지를 불러올 수 없습니다: {self.marker_path}")
        
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink",
            cv2.CAP_GSTREAMER
        )
        if not self.cap.isOpened():
            raise RuntimeError("카메라 열기 실패")

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.step_x = self.frame_width // 3
        self.step_y = self.frame_height // 3

        self.mf_process()

    def mf_process(self):
        self.marker_gray = cv2.cvtColor(self.marker_color, cv2.COLOR_BGR2GRAY)
        self.marker_blur = cv2.GaussianBlur(self.marker_gray, (5, 5), 0)
        _, thresh = cv2.threshold(self.marker_blur, 90, 255, cv2.THRESH_BINARY_INV)

        self.marker_thresh = cv2.resize(thresh, (100, 100))

    def ff_process(self):
        self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.frame_blur = cv2.GaussianBlur(self.frame_gray, (5, 5), 0)
        _, self.frame_thresh = cv2.threshold(self.frame_blur, 90, 255, cv2.THRESH_BINARY_INV)
        self.h, self.w = self.frame_gray.shape

    def tem_match(self):
        for i in range(1, 3):
            cv2.line(self.frame, (i * self.step_x, 0), (i * self.step_x, self.h), (0, 255, 0), 2)
            cv2.line(self.frame, (0, i * self.step_y), (self.w, i * self.step_y), (0, 255, 0), 2)
            
        self.res = cv2.matchTemplate(self.frame_thresh, self.marker_thresh, cv2.TM_CCOEFF_NORMED)
        _, self.max_val, _, self.max_loc = cv2.minMaxLoc(self.res)
        threshold = 0.5
        self.marker_center = None

        if self.max_val > threshold:
            t_h, t_w = self.marker_thresh.shape
            top_left = self.max_loc
            bottom_right = (top_left[0] + t_w, top_left[1] + t_h)
            cv2.rectangle(self.frame, top_left, bottom_right, (0, 0, 255), 2)
            self.marker_center = (top_left[0] + t_w // 2, top_left[1] + t_h // 2)
            cv2.circle(self.frame, self.marker_center, 5, (255, 0, 0), -1)

        if self.marker_center:
            col = self.marker_center[0] // self.step_x + 1
            row = self.marker_center[1] // self.step_y + 1
            position = (row - 1) * 3 + col

            if position == 5:
                center_x_start = self.step_x
                center_y_start = self.step_y
                center_w = self.step_x
                center_h = self.step_y

                sub_step_x = center_w // 3
                sub_step_y = center_h // 3

                relative_x = self.marker_center[0] - center_x_start
                relative_y = self.marker_center[1] - center_y_start

                sub_col = relative_x // sub_step_x + 1
                sub_row = relative_y // sub_step_y + 1
                sub_position = (sub_row - 1) * 3 + sub_col

                for i in range(1, 3):
                    cv2.line(self.frame, (center_x_start + i * sub_step_x, center_y_start),
                             (center_x_start + i * sub_step_x, center_y_start + center_h), (0, 255, 255), 1)
                    cv2.line(self.frame, (center_x_start, center_y_start + i * sub_step_y),
                             (center_x_start + center_w, center_y_start + i * sub_step_y), (0, 255, 255), 1)

                print(f"마커 감지 -> 전체 9분할 {position}번 영역 -> 내부 9분할 {sub_position}번 영역")
            else:
                print(f"마커 감지 -> {position}번 영역")
        else:
            print("마커 없음")

        cv2.imshow("Threshold View", self.frame_thresh)
        cv2.imshow("Marker Tracking - Point Focused", self.frame)

    def run(self):
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break

            self.ff_process()
            self.tem_match()

            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def process_frame(self, frame):
        self.frame = frame
        self.ff_process()
        self.tem_match()
    
        self.debug_frame = self.frame.copy()
    
        return self.marker_center is not None  # 마커 탐지 여부 반환

landing = Landing()
landing.run()
