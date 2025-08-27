import cv2
import numpy as np
import time
from pymavlink import mavutil
import sys

# --- 1. 카메라 파이프라인 함수 정의 ---
def gstreamer_pipeline(
    sensor_id,
    capture_width=1280,
    capture_height=720,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    """
    GStreamer 파이프라인을 생성하여 Jetson의 CSI 카메라를 OpenCV에서 사용할 수 있도록 합니다.
    - sensor-id: 사용할 카메라 센서 (0번 또는 1번)
    - capture_width/height: 센서가 캡처할 해상도
    - display_width/height: 화면에 표시할 해상도
    - framerate: 초당 프레임 수
    - flip-method: 이미지 뒤집기 방법 (0: 없음, 2: 180도 회전 등)
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
    )

# --- 2. 메인 드론 제어 프로그램 ---
if __name__ == "__main__":
    
    # --- FC 연결 및 모드 설정 ---
    print("FC 연결 중...")
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat()
        print("✅ FC 연결 성공")
    except Exception as e:
        print(f"🚨 FC 연결 실패: {e}")
        sys.exit(1)

    print("GUIDED 모드 설정...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED
    )
    time.sleep(1)

    print("시동 걸기...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3)

    print("2m 이륙...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0,
        2  # 목표 고도
    )
    time.sleep(5)
    print("✅ 이륙 완료")
    
    # 15초 호버링 (속도 0 명령)
    print("15초 호버링 시작...")
    for i in range(15):
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
    print("\n호버링 완료")
    
    # --- 착륙 절차 시작: 마커 감지 및 위치 보정 ---
    print("착륙 절차 시작: 마커 감지 및 위치 보정")
    
    pipeline0 = gstreamer_pipeline(sensor_id=0)
    cap = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("GStreamer 파이프라인을 열 수 없습니다. 카메라 연결이나 설정을 확인하세요.")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        sys.exit(1)
        
    MOVE_SPEED_XY = 0.3  # 수평 이동 속도 (m/s)
    MOVE_SPEED_Z = -0.2  # 하강 속도 (m/s), z축은 아래 방향이 양수이므로 음수값 사용
    CENTER_TOLERANCE_PX = 15 # 중앙 허용 오차 (픽셀)
    MIN_MARKER_AREA = 150 # 마커 인식 최소 영역
    LANDING_AREA_THRESHOLD = 50000 # 착륙 판별 임계값 (마커 면적이 이 값 이상이면 착륙)
    last_print_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
            
        # --- 빨간 점 감지 로직 ---
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
        
        marker_center = None
        current_marker_area = 0
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            current_marker_area = cv2.contourArea(largest_contour)
            approx = cv2.approxPolyDP(largest_contour, 0.02 * cv2.arcLength(largest_contour, True), True)
            
            if current_marker_area > MIN_MARKER_AREA and len(approx) > 7:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 7, (0, 255, 0), -1)
                    marker_center = (cx, cy)
        
        # --- 위치 보정 및 하강 명령 전송 ---
        vx, vy, vz = 0, 0, 0
        cmd = "search"
        
        if marker_center is not None:
            # 착륙 지점
            if current_marker_area >= LANDING_AREA_THRESHOLD:
                print("✅ 착륙 지점 도달. 최종 착륙을 시도합니다.")
                break
            
            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2
            x_diff = marker_center[0] - center_x
            y_diff = marker_center[1] - center_y
            
            x_cmd = ""
            y_cmd = ""
            
            if abs(x_diff) > CENTER_TOLERANCE_PX:
                x_cmd = "right" if x_diff > 0 else "left"
            
            if abs(y_diff) > CENTER_TOLERANCE_PX:
                y_cmd = "backward" if y_diff > 0 else "forward"
                
            if x_cmd or y_cmd:
                if "forward" in y_cmd: vy = MOVE_SPEED_XY
                if "backward" in y_cmd: vy = -MOVE_SPEED_XY
                if "left" in x_cmd: vx = -MOVE_SPEED_XY
                if "right" in x_cmd: vx = MOVE_SPEED_XY
                cmd = f"vy={vy:.2f}, vx={vx:.2f}"
            else:
                cmd = "centered" # 중앙 정렬 상태
            
            vz = MOVE_SPEED_Z # 마커가 보이면 계속 하강
            
            # 드론에 이동 명령 전송 (수평 및 수직 속도 포함)
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111, # vx, vy, vz만 제어
                0, 0, 0,
                vx, vy, vz, # vx, vy는 수평 이동, vz는 하강 속도
                0, 0, 0,
                0, 0
            )

        # 디버그용 출력 및 화면 표시
        now = time.time()
        if now - last_print_time > 0.5:
            print(f"현재 명령: {cmd}, 마커 면적: {current_marker_area:.2f}")
            last_print_time = now
            
        cv2.imshow("Landing", frame)
        
        if cv2.waitKey(1) & 0xFF == 27:
            print("사용자 중지 요청.")
            break
    
    # --- 종료 ---
    print("✅ 미션 완료!")
    print("시동 끄기...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0
    )
    time.sleep(1)
    
    if cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()
    print(">> 프로그램 종료.")
