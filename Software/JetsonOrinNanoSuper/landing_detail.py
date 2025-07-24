import cv2
import numpy as np

# --- 마커 이미지 불러오기 ---
marker_path = "/mnt/data/Marker.jpeg"  # 업로드한 파일 경로
marker_color = cv2.imread(marker_path)
if marker_color is None:
    raise FileNotFoundError(f"마커 이미지를 불러올 수 없습니다: {marker_path}")

# --- 마커 전처리 (작은 점 중심) ---
marker_gray = cv2.cvtColor(marker_color, cv2.COLOR_BGR2GRAY)
marker_blur = cv2.GaussianBlur(marker_gray, (5, 5), 0)
_, marker_thresh = cv2.threshold(marker_blur, 90, 255, cv2.THRESH_BINARY_INV)  # 점이 흰색으로

marker_thresh = cv2.resize(marker_thresh, (100, 100))  # 작은 점 인식용 크기

# --- 카메라 설정 ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라 열기 실패")
    exit()

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

step_x = frame_width // 3
step_y = frame_height // 3

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # --- 프레임 전처리 (노이즈 억제 및 이진화) ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, frame_thresh = cv2.threshold(blur, 90, 255, cv2.THRESH_BINARY_INV)

    h, w = gray.shape

    # 1. 전체 화면 3x3 그리드
    for i in range(1, 3):
        cv2.line(frame, (i * step_x, 0), (i * step_x, h), (0, 255, 0), 2)
        cv2.line(frame, (0, i * step_y), (w, i * step_y), (0, 255, 0), 2)

    # 2. 템플릿 매칭
    res = cv2.matchTemplate(frame_thresh, marker_thresh, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(res)
    threshold = 0.4  # 정지 상태에서도 인식되게 낮춤
    marker_center = None

    if max_val > threshold:
        t_h, t_w = marker_thresh.shape
        top_left = max_loc
        bottom_right = (top_left[0] + t_w, top_left[1] + t_h)
        cv2.rectangle(frame, top_left, bottom_right, (0, 0, 255), 2)
        marker_center = (top_left[0] + t_w // 2, top_left[1] + t_h // 2)
        cv2.circle(frame, marker_center, 5, (255, 0, 0), -1)

    # 3. 9분할 및 내부 위치 계산
    if marker_center:
        col = marker_center[0] // step_x + 1
        row = marker_center[1] // step_y + 1
        position = (row - 1) * 3 + col

        if position == 5:
            center_x_start = step_x
            center_y_start = step_y
            center_w = step_x
            center_h = step_y

            sub_step_x = center_w // 3
            sub_step_y = center_h // 3

            relative_x = marker_center[0] - center_x_start
            relative_y = marker_center[1] - center_y_start

            sub_col = relative_x // sub_step_x + 1
            sub_row = relative_y // sub_step_y + 1
            sub_position = (sub_row - 1) * 3 + sub_col

            for i in range(1, 3):
                cv2.line(frame, (center_x_start + i * sub_step_x, center_y_start),
                         (center_x_start + i * sub_step_x, center_y_start + center_h), (0, 255, 255), 1)
                cv2.line(frame, (center_x_start, center_y_start + i * sub_step_y),
                         (center_x_start + center_w, center_y_start + i * sub_step_y), (0, 255, 255), 1)

            print(f"마커 감지 → 전체 9분할 {position}번 영역 → 내부 9분할 {sub_position}번 영역")
        else:
            print(f"마커 감지 → {position}번 영역")
    else:
        print("마커 없음")

    # 디버깅용 화면 출력
    cv2.imshow("Threshold View", frame_thresh)
    cv2.imshow("Marker Tracking - Point Focused", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
