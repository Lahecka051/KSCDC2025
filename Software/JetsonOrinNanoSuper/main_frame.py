import os
import time
import cv2
from flask import Flask, render_template_string, send_from_directory
from threading import Thread
from dronekit import connect
from object_detection_module import Object_Data
from landing_module import Landing
from drone_controller import hover
from throw_module import Servo, compute_throw_parameters

# ===================== Flask 서버 설정 =====================
UPLOAD_FOLDER = "./uploaded_images"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>드론 화재 탐지 이미지</title>
</head>
<body>
    <h1>드론이 촬영한 이미지</h1>
    {% if images %}
        {% for img in images %}
            <h3>{{ img }}</h3>
            <img src="/images/{{ img }}" width="500"><br><br>
        {% endfor %}
    {% else %}
        <p>현재 업로드된 이미지가 없습니다.</p>
    {% endif %}
    <p>※ 5분 후 서버가 자동으로 종료됩니다.</p>
</body>
</html>
"""

@app.route('/')
def index():
    images = sorted(os.listdir(UPLOAD_FOLDER))
    return render_template_string(HTML_TEMPLATE, images=images)

@app.route('/images/<path:filename>')
def get_image(filename):
    return send_from_directory(UPLOAD_FOLDER, filename)

def run_web_server():
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

# ===================== 이미지 저장 함수 =====================
def capture_and_save(frame, filename):
    path = os.path.join(UPLOAD_FOLDER, filename)
    cv2.imwrite(path, frame)
    print(f"[INFO] 이미지 저장 완료: {path}")

# ===================== 메인 함수 =====================
def main():
    # 1) 드론 연결
    print("[INFO] 드론 연결 중...")
    DataRT = connect("udp:127.0.0.1:14550", wait_ready=True)
    print("[INFO] 드론 연결 완료")

    # 2) 객체 탐지 초기화
    cap = cv2.VideoCapture(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )
    detector = Object_Data(cap)

    fire_detected = False
    fire_location = None
    fire_frame = None

    print("[INFO] 순찰 시작")

    # 3) 순찰 중 화재 탐지
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 카메라 프레임 읽기 실패")
            break

        if detector.detect_once(frame):
            print("[ALERT] 화재 탐지됨!")
            fire_detected = True
            fire_location = DataRT.location.global_frame
            fire_frame = frame.copy()
            capture_and_save(fire_frame, "fire_detected.jpg")
            break

        if cv2.waitKey(1) & 0xFF == 27:
            print("[INFO] 순찰 종료")
            cap.release()
            return

    cap.release()

    if not fire_detected:
        print("[INFO] 화재 탐지되지 않음, 순찰 종료")
        return

    # 4) 드론 스테이션으로 복귀 및 착륙
    print("[INFO] 드론 스테이션으로 복귀 중...")
    hover(DataRT, 3)
    landing = Landing()
    landing.run()

    # 5) Flask 웹 서버 실행 (이미지 확인용)
    print("[INFO] 웹 서버 실행 (PC에서 http://<드론_IP>:5000 접속)")
    server_thread = Thread(target=run_web_server, daemon=True)
    server_thread.start()

    # 6) 일정 시간 대기 후 서버 종료
    time.sleep(300)  # 5분간 이미지 확인 가능
    print("[INFO] 5분 경과, 서버 종료")

    # 7) 소화탄 투하 과정 (추가 승인 절차 후 실행 가능)
    theta, v0, d, dz = compute_throw_parameters(
        (fire_location.lat, fire_location.lon, fire_location.alt),
        (fire_location.lat, fire_location.lon, 0),
        initial_angle_deg=45
    )
    print(f"[INFO] 투하 각도: {theta}, 속도: {v0:.2f}, 거리: {d:.2f}")

    servo = Servo(33, freq=50, min_us=500, max_us=2500, angle=180)
    servo.servo_angle(90)
    time.sleep(1.5)
    servo.servo_angle(0)
    servo.cleanup()

if __name__ == "__main__":
    main()
