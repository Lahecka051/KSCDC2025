import cv2
from Object_Detection import Object_Data
from Marker_Landing import Landing

def main():
    # CSI 카메라 GStreamer 파이프라인 설정
    cap = cv2.VideoCapture(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():
        print("카메라 열기 실패")
        return

    # 클래스 객체 초기화
    detector = Object_Data(cap)       # 화재 감지 YOLO
    lander = Landing(cap)             # 마커 기반 착륙

    print("[시스템] 드론 자율 임무 시작")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라 프레임 수신 실패")
            break

        # 1. 화재 탐지
        fire_detected = detector.detect_once(frame)

        if fire_detected:
            print("[탐지] 화재 감지됨! 착륙 루틴으로 전환")
            
            # 2. 마커 착륙 수행
            marker_found = lander.process_frame(frame)

            if marker_found:
                print("[착륙] 🎯 마커 중앙 정렬 및 착륙 시도")
                # 여기서 착륙 명령 전송 가능
                # 예: drone.land()
        else:
            print("[정찰] 화재 없음, 계속 탐색 중")

        # 디버그 창 출력
        cv2.imshow("Detection", detector.debug_frame)
        cv2.imshow("Landing", lander.debug_frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
