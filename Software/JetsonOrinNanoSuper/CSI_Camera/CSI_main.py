import cv2
from landing import Landing
from object_detection import Object_Data

def main():
    print("[1] 마커 기반 착륙 기능 시작")
    landing = Landing()
    landing.run()

    print("[2] YOLO 객체 인식 기능 시작")
    cap = cv2.VideoCapture(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    object_detection = Object_Data(cap)
    object_detection.run_OD()

if __name__ == "__main__":
    main()
