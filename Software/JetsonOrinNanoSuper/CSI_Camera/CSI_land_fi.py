import cv2
import numpy as np
import time
from pipeline import gstreamer_pipeline

class Landing:
    def __init__(self, cap, marker_path = "/home/kscdc2025/Marker.png"):
        self.last_print_time = 0
        self.marker_path = marker_path
        self.marker_color = cv2.imread(self.marker_path)
        if self.marker_color is None:
            raise FileNotFoundError(f"마커 이미지를 불러올 수 없습니다: {self.marker_path}")

        self.cap = cap
        self.marker_size = 100
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.step_x = self.frame_width // 3
        self.step_y = self.frame_height // 3

        self.cmd = None

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

    def detect_red_dot(self):
        # HSV 색공간 변환
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 2개
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 150, 150])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 노이즈 제거
        kernel = np.ones((3,3), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            approx = cv2.approxPolyDP(largest_contour, 0.02 * cv2.arcLength(largest_contour, True), True)
            
            if area > 150 and len(approx) > 7:  # 너무 작은 영역 무시
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # 중심점 표시 (디버그용)
                    cv2.circle(self.frame, (cx, cy), 7, (0, 255, 0), -1)
                    return (cx, cy)
        return None

    def tem_match(self):
        self.cmd = None
        for i in range(1, 3):
            cv2.line(self.frame, (i * self.step_x, 0), (i * self.step_x, self.h), (0, 255, 0), 2)
            cv2.line(self.frame, (0, i * self.step_y), (self.w, i * self.step_y), (0, 255, 0), 2)
            
        self.marker_center = self.detect_red_dot()
        
        if self.marker_center is not None:
            col = self.marker_center[0] // self.step_x + 1
            row = self.marker_center[1] // self.step_y + 1
            position = (row - 1) * 3 + col # 전체 9분할 기준 영역 번호

            outer_cmds = {
                1: ["level", "forward_left", 0, 10],
                2: ["level", "forward", 0, 10],
                3: ["level", "forward_right", 0, 10],
                4: ["level", "left", 0, 10],
                6: ["level", "right", 0, 10],
                7: ["level", "backward_left", 0, 10],
                8: ["level", "backward", 0, 10],
                9: ["level", "backward_right", 0, 10]
            }

            if position in outer_cmds:
                self.cmd = outer_cmds[position]
                print(f"마커 감지 -> 전체 9분할 {position}번 영역/ 명령: {self.cmd}")

            elif position == 5:
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
                sub_position = (sub_row - 1) * 3 + sub_col   # 중앙 영역 9분할 내부 영역 판단

                inner_cmds = {
                    1: ["level", "forward_left", 0, 5],
                    2: ["level", "forward", 0, 5],
                    3: ["level", "forward_right", 0, 5],
                    4: ["level", "left", 0, 5],
                    5: ["level", "stop", 0, 5],
                    6: ["level", "right", 0, 5],
                    7: ["level", "backward_left", 0, 5],
                    8: ["level", "backward", 0, 5],
                    9: ["level", "backward_right", 0, 5]
                }

                self.cmd = inner_cmds.get(sub_position)
                print(f"마커 감지 -> 전체 9분할 5번 영역 -> 내부 9분할 {sub_position}번 영역 / 명령: {self.cmd}")

                for i in range(1, 3):
                    cv2.line(self.frame, (center_x_start + i * sub_step_x, center_y_start),
                             (center_x_start + i * sub_step_x, center_y_start + center_h), (0, 255, 255), 1)
                    cv2.line(self.frame, (center_x_start, center_y_start + i * sub_step_y),
                             (center_x_start + center_w, center_y_start + i * sub_step_y), (0, 255, 255), 1)
            
            else:
                print(f"마커 감지 -> 위치 알 수 없음 (position: {position})")
        else:
            print("마커 없음")

        cv2.imshow("Frame with Red Dot Detection", self.frame)

        return self.cmd

    def run(self):
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break

            self.ff_process()
            self.tem_match()

            now = time.time()
            if now - self.last_print_time > 0.5:
                print("현재 명령:", self.cmd)
                self.last_print_time = now

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

"""
def gstreamer_pipeline(
    sensor_id,
    capture_width=1280,
    capture_height=720,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
    )
"""

if __name__ == "__main__":
    from pipeline import gstreamer_pipeline

    pipeline0 = gstreamer_pipeline(sensor_id=0)
    cap = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("GStreamer 파이프라인을 열 수 없습니다. 카메라 연결이나 설정을 확인하세요.")
    else:
        landing = Landing(cap)
        landing.run()
