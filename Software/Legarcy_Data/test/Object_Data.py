from ultralytics import YOLO
import cv2

class Object_Data:
    def __init__(self,cap):
        self.model = YOLO("best.pt")
        self.cap = cap

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.w1, self.w2 = self.frame_width // 3, 2 * self.frame_width // 3
        self.h1, self.h2 = self.frame_height // 3, 2 * self.frame_height // 3

        self.class_names = self.model.names
        self.cmd = []

    def get_position_label(self,x,y):
        if y < self.h1:
            vertical = "위"
        elif y < self.h2:
            vertical = "가운데"
        else:
            vertical = "아래"

        if x < self.w1:
            horizontal = "왼쪽"
        elif x < self.w2:
            horizontal = "가운데"
        else:
            horizontal = "오른쪽"

        if (vertical == "위" and horizontal == "왼쪽"):
            self.cmd = ["level","forward_left",0,10]

        elif (vertical == "위" and horizontal == "가운데"):
            self.cmd = ["level","forward",0,10]
        

        elif (vertical == "위" and horizontal == "오른쪽"):
            self.cmd = ["level","forward_right",0,10]

        elif (vertical == "가운데" and horizontal == "왼쪽"):
            self.cmd = ["level","left",0,10]

        elif (vertical == "가운데" and horizontal == "가운데"):
            self.cmd = ["level","stop",0,10]

        elif (vertical == "가운데" and horizontal == "오른쪽"):
            self.cmd = ["level","right",0,10]

        elif (vertical == "아래" and horizontal == "왼쪽"):
            self.cmd = ["level","backward_left",0,10]

        elif (vertical == "아래" and horizontal == "가운데"):
            self.cmd = ["level","backward",0,10]

        elif (vertical == "아래" and horizontal == "아래"):
            self.cmd = ["level","backwarod_right",0,10]
        
        return self.cmd

    def run_OD(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            scale_factor = 1.5
            resized_frame = cv2.resize(frame,None,fx=scale_factor,fy=scale_factor)
            results = self.model.predict(resized_frame, imgsz=1280, conf=0.4, verbose=False)
            annotated_frame = results[0].plot()

            for box in results[0].boxes:
                # 클래스 ID, 이름
                cls_id = int(box.cls[0])
                class_name = self.class_names[cls_id]

                # 중심 좌표
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                label = self.get_position_label(center_x, center_y)
                print(f"→ {class_name}:",label)

                # 시각화
                cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(annotated_frame, f"{class_name}", (center_x, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 결과 출력
            cv2.imshow("YOLOv8 Detection with Class & Position", annotated_frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()


cap = cv2.VideoCapture(0)
ob = Object_Data(cap)
ob.run_OD()
        
