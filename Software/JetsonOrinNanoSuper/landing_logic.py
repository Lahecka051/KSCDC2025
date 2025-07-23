import cv2
import numpy as np
import time

marker_path = "파일 경로" 
marker = cv2.imread(marker_path, cv2.IMREAD_GRAYSCALE) # 이미지 그레이 스케일로 읽음
if marker is None:
    raise FileNotFoundError(f"마커 이미지를 불러올 수 없습니다: {marker_path}")

marker = cv2.resize(marker, (100, 100)) # 테스트 이미지가 웹캠 해상도보다 커서 크기 조정, 템플릿 너무 크면 매칭 속도 느려짐

cap = cv2.VideoCapture(0) # 웹캠 연결 객체 생성(0은 기본 카메라)
if not cap.isOpened():
    raise RuntimeError("웹캠을 열 수 없습니다.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 현재 프레임을 그레이스케일 이미지로 변환(매칭 위함)
    h, w = gray.shape # 프레임 가로(h), 세로(w) 픽셀 수 가져옴

    # 1. 전체 화면 3x3 그리기
    step_x = w // 3
    step_y = h // 3
    for i in range(1, 3):
        cv2.line(frame, (i * step_x, 0), (i * step_x, h), (0, 255, 0), 2)
        cv2.line(frame, (0, i * step_y), (w, i * step_y), (0, 255, 0), 2)

    res = cv2.matchTemplate(gray, marker, cv2.TM_CCOEFF_NORMED) # res = 유사도 맵(0~1), 값 높을수록 높은 매칭
    _, max_val, _, max_loc = cv2.minMaxLoc(res)                 # 최대 유사도(max_val), 그 위치(max_loc)
    threshold = 0.6                                             # max_val > 0.6(threshold)이면 마커 인식되었다고 판단
    marker_center = None                                        # 기본값 = 인식 실패(마커 없음)

    if max_val > threshold:
        t_h, t_w = marker.shape
        top_left = max_loc
        bottom_right = (top_left[0] + t_w, top_left[1] + t_h)
        cv2.rectangle(frame, top_left, bottom_right, (0, 0, 255), 2)
        marker_center = (top_left[0] + t_w // 2, top_left[1] + t_h // 2)
        cv2.circle(frame, marker_center, 5, (255, 0, 0), -1)

    if marker_center:
        # 2. 전체 3x3에서 위치 계산
        col = marker_center[0] // step_x + 1
        row = marker_center[1] // step_y + 1
        position = (row - 1) * 3 + col

        if position == 5:
            # 3. 5번 영역(중앙 영역) 좌표 계산
            center_x_start = step_x
            center_y_start = step_y
            center_w = step_x
            center_h = step_y

            # 4. 5번 영역을 다시 3x3로 나누기
            sub_step_x = center_w // 3
            sub_step_y = center_h // 3

            # 마커가 5번 영역 내 좌표 (상대 좌표)
            relative_x = marker_center[0] - center_x_start
            relative_y = marker_center[1] - center_y_start

            sub_col = relative_x // sub_step_x + 1
            sub_row = relative_y // sub_step_y + 1
            sub_position = (sub_row - 1) * 3 + sub_col

            # 5. 5번 영역 9분할 그리기 (녹색보다 조금 밝은 색으로 표시)
            for i in range(1, 3):
                # 세로선
                cv2.line(frame, (center_x_start + i * sub_step_x, center_y_start),
                         (center_x_start + i * sub_step_x, center_y_start + center_h), (0, 255, 255), 1)
                # 가로선
                cv2.line(frame, (center_x_start, center_y_start + i * sub_step_y),
                         (center_x_start + center_w, center_y_start + i * sub_step_y), (0, 255, 255), 1)

            print(f"마커 감지 → 전체 9분할 {position}번 영역 → 내부 9분할 {sub_position}번 영역")
        else:
            print(f"마커 감지 → {position}번 영역")
    else:
        print("마커 없음")

    cv2.imshow("Marker Tracking (9-Split + 5번 9-Split)", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

    time.sleep(0.2)

cap.release()
cv2.destroyAllWindows()
