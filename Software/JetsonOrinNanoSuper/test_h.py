import cv2
import numpy as np

# 카메라 열기
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라 열기 실패")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 전처리
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)

    # 윤곽선 추출
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:  # 너무 작은 건 무시
            continue

        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = w / float(h)

        # H는 일반적으로 세로 직사각형 형태
        if 0.3 < aspect_ratio < 0.7 and w > 20 and h > 50:
            # ROI 영역 자르기
            roi = gray[y:y+h, x:x+w]
            _, roi_bin = cv2.threshold(roi, 127, 255, cv2.THRESH_BINARY_INV)

            # 다시 윤곽선 추출하여 내부 구조 파악
            inner_contours, _ = cv2.findContours(roi_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(inner_contours) >= 3:
                # 대충 수직 2개, 수평 1개 → H 구조로 간주
                detected = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "H Detected", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    if not detected:
        cv2.putText(frame, "No H Detected", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("H Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
