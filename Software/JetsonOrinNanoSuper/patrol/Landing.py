import cv2
import numpy as np
import time

class Landing:
    def __init__(self, cap, marker_path="/home/kscdc2025/Marker.png"):
        self.last_print_time = 0
        self.marker_path = marker_path
        self.marker_color = cv2.imread(self.marker_path)
        if self.marker_color is None:
            raise FileNotFoundError(f"마커 이미지를 불러올 수 없습니다: {self.marker_path}")

        self.cap = cap
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

    def ff_process(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)
        _, frame_thresh = cv2.threshold(frame_blur, 90, 255, cv2.THRESH_BINARY_INV)
        return frame_thresh, frame_gray.shape

    def detect_red_dot(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 150, 150])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3,3), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            approx = cv2.approxPolyDP(largest_contour, 0.02 * cv2.arcLength(largest_contour, True), True)
            if area > 150 and len(approx) > 7:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
        return None

    def process_frame(self, frame):
        self.frame = frame.copy()
        frame_thresh, (self.h, self.w) = self.ff_process(self.frame)
        self.marker_center = self.detect_red_dot(self.frame)

        self.cmd = None
        if self.marker_center is not None:
            col = self.marker_center[0] // self.step_x + 1
            row = self.marker_center[1] // self.step_y + 1
            position = (row - 1) * 3 + col

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
                sub_position = (sub_row - 1) * 3 + sub_col

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

        return self.cmd, self.marker_center, self.frame.copy()

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            cmd, marker_center, debug_frame = self.process_frame(frame)

            now = time.time()
            if now - self.last_print_time > 0.5:
                self.last_print_time = now

            cv2.imshow("Landing Detection", debug_frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()
