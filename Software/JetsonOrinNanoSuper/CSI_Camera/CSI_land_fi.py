import cv2
import numpy as np
import time
from pipeline import gstreamer_pipeline

# GStreamer 파이프라인 함수를 클래스 외부에 정의합니다.
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

class Landing:
    def __init__(self, display_width=960, display_height=540):
        self.frame_width = display_width
        self.frame_height = display_height
        self.last_print_time = 0
        self.last_command = [0, 0, 0, 0] # 마지막 명령 저장

    def detect_red_dot(self, frame):
        """프레임에서 가장 큰 빨간 점을 감지하고 중심 좌표와 면적을 반환"""
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
                    return (cx, cy, area)
        return None

    def get_control_command(self, frame):
        """
        카메라 프레임을 분석하여 드론 제어 명령을 반환.
        명령 형식: [위아래, 전후, 좌우, 회전]
        """
        
        # 화면 중앙 좌표
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        
        # 마커 탐지
        marker_info = self.detect_red_dot(frame)
        
        # 기본 명령 (마커 없을 시)
        # 위아래(수평 유지), 전후(정지), 좌우(정지), 회전(정지)
        command = [0, 0, 0, 0] 
        
        if marker_info:
            marker_center_x, marker_center_y, area = marker_info
            
            # 디버그용 중심점 표시
            cv2.circle(frame, (marker_center_x, marker_center_y), 7, (0, 255, 0), -1)
            
            # 마커의 상대적 위치
            x_diff = marker_center_x - center_x
            y_diff = marker_center_y - center_y
            
            # 명령 계산
            # 정수형으로 변환하기 위한 스케일링 팩터
            SCALE = 15 
            
            # 위아래 (하강) - 마커 감지 시 하강 명령
            command[0] = -0.2
            
            # 전후 (+: 전진, -: 후진) - Y축 오차에 비례하여 제어
            # 마커가 아래(y_diff > 0)에 있으면 드론은 후진해야 함
            command[1] = -(y_diff // SCALE)
            
            # 좌우 (+: 좌측, -: 우측) - X축 오차에 비례하여 제어
            # 마커가 오른쪽(x_diff > 0)에 있으면 드론은 우측으로 이동해야 함
            command[2] = -(x_diff // SCALE)
            
            # 회전
            command[3] = 0
            
            # 디버그 메시지 출력 (0.5초 간격)
            now = time.time()
            if now - self.last_print_time > 0.5:
                print(f"마커 감지 | 오차: x={x_diff}, y={y_diff} | 명령: {command}")
                self.last_print_time = now
        else:
            # 마커 미감지 시, 마지막 명령을 유지하거나 정지
            # 여기서는 편의상 정지 명령을 보냅니다.
            print("마커 없음. 드론 정지 또는 호버링")
            
        return command

    def run_main_loop(self):
        """
        테스트를 위한 메인 루프 (실제 드론 제어 코드에 통합되어야 함)
        """
        pipeline0 = gstreamer_pipeline(sensor_id=0)
        cap = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print("GStreamer 파이프라인을 열 수 없습니다. 카메라 연결이나 설정을 확인하세요.")
            return

        print("카메라를 시작합니다. 'ESC' 키를 눌러 종료하세요.")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break
                
            # 제어 명령 얻기
            command = self.get_control_command(frame)
            
            # 이 부분에서 얻은 'command' 리스트를 드론 제어 코드로 전송하면 됩니다.
            # 예: master.mav.set_position_target_local_ned_send(...)
            
            # 디버그용 화면 표시
            cv2.imshow("Drone Landing", frame)
            
            if cv2.waitKey(1) & 0xFF == 27:
                print("사용자 중지 요청.")
                break
        
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # 클래스 인스턴스화
    landing_controller = Landing()
    
    # 메인 루프 실행
    landing_controller.run_main_loop()
