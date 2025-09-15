import cv2
import numpy as np
import platform
import time

# 빨간 점이 y축에 있다고 판단하는 허용 오차 (픽셀)
X_AXIS_TOLERANCE = 5

def gstreamer_pipeline(
    framerate=30,
    flip_method=0,
):
    """
    라즈베리 파이 CSI 카메라용 GStreamer 파이프라인
    - 기본 해상도를 사용하고, 이후 프레임을 크롭하여 정사각형으로 만듭니다.
    """
    return (
        f"libcamerasrc ! "
        f"video/x-raw, framerate=(fraction){framerate}/1 ! "
        f"videoflip method={flip_method} ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )

def setup_camera():
    """
    운영체제에 따라 카메라를 초기화합니다.
    """
    os_type = platform.system()
    
    if os_type == 'Darwin': # macOS
        print("💻 macOS 웹캠으로 테스트를 시작합니다. (정사각형으로 크롭됩니다)")
        cap = cv2.VideoCapture(0)
    elif os_type == 'Linux': # 라즈베리 파이 OS
        print("🖥️ 라즈베리 파이 CSI 카메라로 테스트를 시작합니다. (정사각형으로 크롭됩니다)")
        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    else:
        raise NotImplementedError("지원하지 않는 운영체제입니다.")
        
    if not cap.isOpened():
        raise IOError("카메라를 열 수 없습니다. 연결이나 설정을 확인하세요.")
        
    return cap

def detect_red_dot(frame):
    """
    프레임에서 가장 큰 빨간 점을 감지하고 중심 좌표를 반환합니다.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 빨간색의 HSV 범위는 0 근처와 180 근처에 걸쳐 있습니다.
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # 노이즈 제거 및 컨투어 찾기
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        # 가장 큰 컨투어 찾기 (가장 큰 빨간 점)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 면적 필터링
        if cv2.contourArea(largest_contour) > 50:
            # 중심 좌표 계산
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                return (center_x, center_y)
                
    return None

def run_camera_test():
    """
    카메라 테스트를 실행하는 메인 함수
    """
    cap = setup_camera()
    
    last_print_time = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다. 프로그램을 종료합니다.")
            break

        # --- 카메라 프레임을 정사각형으로 크롭하는 로직 ---
        h, w, _ = frame.shape
        size = min(h, w)
        x_start = (w - size) // 2
        y_start = (h - size) // 2
        square_frame = frame[y_start:y_start+size, x_start:x_start+size]
        # --------------------------------------------------
        
        center = (size // 2, size // 2)
        dot_center = detect_red_dot(square_frame)
        
        is_matched = False
        if dot_center:
            # 감지된 점의 중심 좌표 (0,0) 기준
            dot_x_relative = dot_center[0] - center[0]
            dot_y_relative = dot_center[1] - center[1]
            
            # 매칭 조건: x축이 0에 가깝고, y축은 방향에 관계없이 인식
            if abs(dot_x_relative) < X_AXIS_TOLERANCE:
                is_matched = True

            # 인식된 점에 원 그리기 (디버깅용)
            cv2.circle(square_frame, dot_center, 10, (0, 255, 0), 2)
            cv2.putText(square_frame, f'({dot_x_relative}, {dot_y_relative})', (dot_center[0] + 20, dot_center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 화면에 결과 표시 및 터미널 출력
        if is_matched:
            cv2.putText(square_frame, "MATCHED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            if time.time() - last_print_time > 1: # 1초에 한 번만 출력
                print("매칭됨")
                last_print_time = time.time()
        else:
            cv2.putText(square_frame, "NOT MATCHED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Camera Test", square_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run_camera_test()
