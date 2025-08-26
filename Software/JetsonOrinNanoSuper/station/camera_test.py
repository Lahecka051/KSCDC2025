# camera_module.py

import cv2
import numpy as np
import time

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    """
    라즈베리 파이 CSI 카메라용 GStreamer 파이프라인
    - libcamerasrc: Raspberry Pi OS에서 권장되는 최신 카메라 소스
    """
    return (
        f"libcamerasrc ! "
        f"video/x-raw, width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"videoflip method={flip_method} ! "
        f"videoconvert ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGR ! appsink"
    )

class Docking:
    def __init__(self, marker_path="/home/kscdc2025/Marker.png"):
        self.marker_path = marker_path
        self.marker_color = cv2.imread(self.marker_path)
        if self.marker_color is None:
            # 이 마커 이미지는 더 이상 사용되지 않지만, 경로 확인을 위해 유지합니다.
            raise FileNotFoundError(f"마커 이미지를 불러올 수 없습니다: {self.marker_path}")
        
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise IOError("GStreamer 파이프라인을 열 수 없습니다. 카메라 연결이나 설정을 확인하세요.")

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.step_x = self.frame_width // 3
        self.step_y = self.frame_height // 3
        self.cmd = None
        self.last_area = 0

    def detect_circle(self):
        ret, frame = self.cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            return None, None
        
        self.frame = frame
        self.h, self.w = self.frame.shape[:2]
        
        # 흑백으로 변환 및 블러 처리
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 외곽선 검출 (테두리 인식)
        edges = cv2.Canny(gray_blur, 50, 150)
        
        # 컨투어 찾기. RECT_CCOMP는 외곽 컨투어와 내부 컨투어를 모두 찾음
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None

        # 원형과 속이 빈 형태(홀)를 가진 컨투어 찾기
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < 50: # 작은 노이즈 무시
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            # 원형에 가까운지 확인 (원형성)
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if 0.7 <= circularity <= 1.2:
                # 내부 컨투어(홀)가 있는지 확인
                if hierarchy[0][i][2] != -1:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 중심점 표시
                        cv2.circle(self.frame, (cx, cy), 7, (0, 255, 0), -1)
                        self.last_area = area # 거리 계산을 위해 면적 저장
                        return (cx, cy), area
        
        return None, None

    def calculate_distance(self):
        """
        원의 면적을 기반으로 거리를 계산합니다. (거리 = K / sqrt(면적))
        - K는 시스템의 실제 환경에 맞춰 튜닝해야 하는 상수입니다.
        """
        if self.last_area > 0:
            # K 값은 사용 환경에 맞춰 직접 계산하여 수정해야 합니다. (예: 100000)
            K_CONSTANT = 100000
            distance_mm = K_CONSTANT / np.sqrt(self.last_area)
            return distance_mm
        return None

    def get_command(self):
        self.cmd = None
        marker_center, area = self.detect_circle()

        if marker_center is not None:
            col = marker_center[0] // self.step_x + 1
            row = marker_center[1] // self.step_y + 1
            position = (row - 1) * 3 + col

            if position == 5:
                # 중앙 영역 내에서 세부 정렬
                center_x_start = self.step_x
                center_y_start = self.step_y
                sub_step_x = center_x_start // 3
                sub_step_y = center_y_start // 3
                relative_x = marker_center[0] - center_x_start
                relative_y = marker_center[1] - center_y_start
                sub_col = relative_x // sub_step_x + 1
                sub_row = relative_y // sub_step_y + 1
                sub_position = (sub_row - 1) * 3 + sub_col
                
                if sub_position == 5:
                    self.cmd = "stop"
                else:
                    self.cmd = ["forward_left", "forward", "forward_right",
                                "left", "stop", "right",
                                "backward_left", "backward", "backward_right"][sub_position - 1]

            else:
                self.cmd = ["forward_left", "forward", "forward_right",
                            "left", "stop", "right",
                            "backward_left", "backward", "backward_right"][position - 1]
        
        return self.cmd
    
    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
