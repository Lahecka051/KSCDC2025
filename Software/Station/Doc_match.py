# camera_module.py

import cv2
import numpy as np
import platform

class Docking:
    def __init__(self):
        # 빨간 점이 y축에 있다고 판단하는 허용 오차 (픽셀)
        self.X_AXIS_TOLERANCE = 5
        # 카메라로 인식할 빨간 점의 최소/최대 면적 (픽셀 단위)
        self.MIN_RED_DOT_AREA = 50
        self.MAX_RED_DOT_AREA = 500
        # 원형도(Circularity) 필터링 임계값 (0 ~ 1, 원에 가까울수록 1)
        self.CIRCULARITY_THRESHOLD = 0.8
        
        self.cap = self.setup_camera()

    def gstreamer_pipeline(self, framerate=30, flip_method=0):
        """라즈베리 파이 CSI 카메라용 GStreamer 파이프라인"""
        return (
            f"libcamerasrc ! "
            f"video/x-raw, framerate=(fraction){framerate}/1 ! "
            f"videoflip method={flip_method} ! "
            f"videoconvert ! "
            f"video/x-raw, format=(string)BGR ! appsink"
        )

    def setup_camera(self):
        """운영체제에 따라 카메라를 초기화하고 반환합니다."""
        os_type = platform.system()
        
        if os_type == 'Darwin': # macOS
            print("💻 macOS 웹캠으로 테스트를 시작합니다.")
            cap = cv2.VideoCapture(0)
        elif os_type == 'Linux': # 라즈베리 파이 OS
            print("🖥️ 라즈베리 파이 CSI 카메라로 테스트를 시작합니다.")
            cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        else:
            raise NotImplementedError("지원하지 않는 운영체제입니다.")
            
        if not cap.isOpened():
            raise IOError("카메라를 열 수 없습니다. 연결이나 설정을 확인하세요.")
            
        return cap

    def detect_red_dot(self, frame):
        """
        프레임에서 가장 적합한 빨간 마커 도트를 감지하고 중심 좌표를 반환합니다.
        - 강화된 필터를 적용하여 정확도를 높였습니다.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
        
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_match_center = None
        best_match_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.MIN_RED_DOT_AREA < area < self.MAX_RED_DOT_AREA:
                (x_circ, y_circ), radius = cv2.minEnclosingCircle(contour)
                circularity = 0
                if radius > 0:
                    circularity = (4 * np.pi * area) / (np.pi * radius * radius)

                if circularity > self.CIRCULARITY_THRESHOLD:
                    x_rect, y_rect, w_rect, h_rect = cv2.boundingRect(contour)
                    aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                    
                    if 0.8 < aspect_ratio < 1.2:
                        if area > best_match_area:
                            M = cv2.moments(contour)
                            if M["m00"] != 0:
                                center_x = int(M["m10"] / M["m00"])
                                center_y = int(M["m01"] / M["m00"])
                                best_match_center = (center_x, center_y)
                                best_match_area = area
        return best_match_center

    def get_coordinates(self):
        """카메라 프레임을 읽어와 빨간 점의 상대적 좌표를 반환합니다."""
        ret, frame = self.cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다. 카메라를 다시 확인하세요.")
            return None, None, False

        h, w, _ = frame.shape
        size = min(h, w)
        x_start = (w - size) // 2
        y_start = (h - size) // 2
        square_frame = frame[y_start:y_start+size, x_start:x_start+size]
        
        dot_center = self.detect_red_dot(square_frame)
        
        is_matched = False
        dot_x_relative = None
        dot_y_relative = None

        if dot_center:
            center = (size // 2, size // 2)
            dot_x_relative = dot_center[0] - center[0]
            dot_y_relative = dot_center[1] - center[1]
            
            if abs(dot_x_relative) < self.X_AXIS_TOLERANCE:
                is_matched = True

            # 디버깅용 화면 표시
            cv2.circle(square_frame, dot_center, 10, (0, 255, 0), 2)
            cv2.putText(square_frame, f'({dot_x_relative}, {dot_y_relative})', (dot_center[0] + 20, dot_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("Camera View", square_frame)
        cv2.waitKey(1)
        
        return dot_x_relative, dot_y_relative, is_matched

    def cleanup(self):
        """카메라 자원을 해제합니다."""
        self.cap.release()
        cv2.destroyAllWindows()
