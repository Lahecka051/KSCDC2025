from ultralytics import YOLO
import cv2

# 모델 로딩
model = YOLO("best.pt")

# 노트북 카메라 열기
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라 열기 실패")
    exit()

# 화면 크기
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
w1, w2 = frame_width // 3, 2 * frame_width // 3
h1, h2 = frame_height // 3, 2 * frame_height // 3

# 클래스 이름 가져오기
class_names = model.names  # {0: 'person', 1: 'car', ...}

def get_position_label(x, y):
    if y < h1:
        vertical = "위"
    elif y < h2:
        vertical = "가운데"
    else:
        vertical = "아래"

    if x < w1:
        horizontal = "왼쪽"
    elif x < w2:
        horizontal = "가운데"
    else:
        horizontal = "오른쪽"

    return f"{vertical}-{horizontal}"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, imgsz=640, conf=0.5, verbose=False)
    annotated_frame = results[0].plot()

    for box in results[0].boxes:
        # 클래스 ID, 이름
        cls_id = int(box.cls[0])
        class_name = class_names[cls_id]

        # 중심 좌표
        x1, y1, x2, y2 = box.xyxy[0]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        label = get_position_label(center_x, center_y)
        print(f"→ {class_name}: {label}")

        # 시각화
        cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.putText(annotated_frame, f"{class_name}: {label}", (center_x, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # 결과 출력
    cv2.imshow("YOLOv8 Detection with Class & Position", annotated_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
