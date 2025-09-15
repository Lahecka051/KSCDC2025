import threading
import time
import math
import os
from datetime import datetime

import cv2
from ultralytics import YOLO
from flask import Flask, send_from_directory, jsonify, request

# Jetson GPIO 라이브러리, dronekit 등은 실제 환경에 맞게 import 필요
import Jetson.GPIO as GPIO
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ===================== 환경 변수 및 설정 =====================

# Jetson GPIO 서보 설정
SERVO_PIN = 33
FREQ = 50
MIN_US = 500
MAX_US = 2500
ANGLE_RANGE = 180

# Flask 서버 포트
FLASK_PORT = 5000

# 저장 폴더
IMG_SAVE_DIR = "./captured_fire_images"
os.makedirs(IMG_SAVE_DIR, exist_ok=True)

# 드론 연결 (Pixhawk)
# 실제 연결 문자열로 교체 필요 (예: '/dev/ttyACM0', baud=115200)
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# ===================== 서보 제어 클래스 =====================

class Servo:
    def __init__(self, pin, freq=50, min_us=500, max_us=2500, angle=180):
        self.pin = pin
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us
        self.angle = angle

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)

    def servo_angle(self, degrees):
        degrees = max(0, min(self.angle, degrees))
        total_range = self.max_us - self.min_us
        us = self.min_us + (total_range * degrees / self.angle)
        duty_cycle = (us / 20000) * 100
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

# ===================== 거리 계산 함수 =====================

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # m
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ===================== 투척 각도/속도 계산 =====================

def compute_throw_parameters(drone_pos, fire_pos, initial_angle_deg=45):
    d = haversine_distance(drone_pos[0], drone_pos[1], fire_pos[0], fire_pos[1])
    dz = drone_pos[2] - fire_pos[2]
    theta = math.radians(initial_angle_deg)
    denominator = 2 * (math.cos(theta)**2) * (d * math.tan(theta) - dz)
    if denominator <= 0:
        raise ValueError("투척 각도가 너무 낮거나 목표 지점이 너무 가까움")
    v0 = math.sqrt((9.81 * d**2) / denominator)
    return initial_angle_deg, v0, d, dz

# ===================== 드론 비행 제어 함수 =====================

def execute_flight_command(vehicle, vertical_mode, angle, speed, duration):
    if vehicle.mode.name != "GUIDED":
        print("[ERROR] Vehicle must be in GUIDED mode.")
        return

    if vertical_mode == 'ascend':
        vz_speed = -1.0
    elif vertical_mode == 'descend':
        vz_speed = 1.0
    else:
        vz_speed = 0.0

    angle_rad = math.radians(angle)
    vx = speed * math.cos(angle_rad)
    vy = speed * math.sin(angle_rad)

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz_speed, 0, 0, 0, 0, 0
    )

    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

    # Hover stop
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(stop_msg)

# ===================== 객체 탐지 클래스 =====================

class Object_Data:
    def __init__(self, cap):
        self.model = YOLO("best.pt")
        self.cap = cap
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.class_names = self.model.names

    def detect_fire_once(self, frame):
        results = self.model.predict(frame, imgsz=1280, conf=0.4, verbose=False)
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if self.class_names[cls_id] == "fire":
                return True
        return False

    def capture_fire_image(self):
        ret, frame = self.cap.read()
        if not ret:
            print("[ERROR] Camera frame read failed.")
            return None
        if self.detect_fire_once(frame):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"fire_{timestamp}.jpg"
            filepath = os.path.join(IMG_SAVE_DIR, filename)
            cv2.imwrite(filepath, frame)
            print(f"[INFO] Fire detected! Image saved: {filepath}")
            return filepath, frame
        return None, None

# ===================== 착륙 관련 (마커 탐지) 클래스 =====================

class Landing:
    def __init__(self, marker_path="Marker.png"):
        self.marker_path = marker_path
        self.marker_color = cv2.imread(self.marker_path)
        if self.marker_color is None:
            raise FileNotFoundError(f"마커 이미지 로드 실패: {self.marker_path}")

        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink",
            cv2.CAP_GSTREAMER
        )
        if not self.cap.isOpened():
            raise RuntimeError("카메라 열기 실패")

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.step_x = self.frame_width // 3
        self.step_y = self.frame_height // 3
        self.marker_center = None

    def detect_marker(self, frame):
        marker_gray = cv2.cvtColor(self.marker_color, cv2.COLOR_BGR2GRAY)
        marker_blur = cv2.GaussianBlur(marker_gray, (5,5), 0)
        _, marker_thresh = cv2.threshold(marker_blur, 90, 255, cv2.THRESH_BINARY_INV)
        marker_thresh = cv2.resize(marker_thresh, (100, 100))

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_blur = cv2.GaussianBlur(frame_gray, (5,5), 0)
        _, frame_thresh = cv2.threshold(frame_blur, 90, 255, cv2.THRESH_BINARY_INV)

        res = cv2.matchTemplate(frame_thresh, marker_thresh, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        threshold = 0.5

        if max_val > threshold:
            t_h, t_w = marker_thresh.shape
            top_left = max_loc
            bottom_right = (top_left[0] + t_w, top_left[1] + t_h)
            self.marker_center = (top_left[0] + t_w // 2, top_left[1] + t_h // 2)
            return True, frame, self.marker_center
        else:
            self.marker_center = None
            return False, frame, None

# ===================== Flask 웹서버 =====================

app = Flask(__name__)
last_captured_image = None

@app.route("/")
def index():
    global last_captured_image
    if last_captured_image and os.path.exists(last_captured_image):
        filename = os.path.basename(last_captured_image)
        return send_from_directory(IMG_SAVE_DIR, filename)
    else:
        return "화재 이미지가 아직 없습니다."

def start_flask():
    app.run(host="0.0.0.0", port=FLASK_PORT)

# ===================== 소화탄 자동 장착 자리 (여백) =====================

def auto_attach_extinguisher():
    """
    TODO: 소화탄 자동 장착 하드웨어 연동 코드 삽입 예정
    """
    print("[INFO] 소화탄 자동 장착 시스템 작동 (예비 함수)")
    time.sleep(3)  # 임시 대기

# ===================== 소화탄 투하 제어 =====================

def drop_extinguisher(servo: Servo):
    """
    서보 모터로 소화탄 뚜껑 열기/닫기
    """
    print("[INFO] 소화탄 투하 - 뚜껑 열기")
    servo.servo_angle(90)
    time.sleep(1.5)
    print("[INFO] 소화탄 투하 - 뚜껑 닫기")
    servo.servo_angle(0)
    time.sleep(0.5)

# ===================== 메인 제어 로직 =====================

def main():
    global last_captured_image
    cap = cv2.VideoCapture(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink",
        cv2.CAP_GSTREAMER
    )

    # 객체 탐지, 착륙 객체 생성
    object_detector = Object_Data(cap)
    landing_detector = Landing()

    # 서보 초기화
    servo = Servo(SERVO_PIN, freq=FREQ, min_us=MIN_US, max_us=MAX_US, angle=ANGLE_RANGE)

    # Flask 서버 스레드 실행
    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()
    print(f"[INFO] Flask 서버가 http://<Jetson_IP>:{FLASK_PORT} 에서 실행 중")

    # 드론 상태 변수
    fire_detected = False
    fire_gps = None
    fire_image_path = None

    # 반복 플래그
    continue_fire_drop = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] 카메라 프레임 읽기 실패")
                break

            # 화재 탐지
            if not fire_detected:
                is_fire = object_detector.detect_fire_once(frame)
                if is_fire:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"fire_{timestamp}.jpg"
                    filepath = os.path.join(IMG_SAVE_DIR, filename)
                    cv2.imwrite(filepath, frame)
                    fire_detected = True
                    fire_image_path = filepath

                    # TODO: 실제 GPS 읽는 코드로 변경 필요
                    fire_gps = (vehicle.location.global_relative_frame.lat,
                                vehicle.location.global_relative_frame.lon,
                                vehicle.location.global_relative_frame.alt)

                    print(f"[INFO] 화재 감지! 이미지 저장: {filepath}")
                    print(f"[INFO] 화재 위치 기록: {fire_gps}")

            # 화재 감지 후 착륙 및 복귀 루틴
            if fire_detected and not continue_fire_drop:
                print("[INFO] 화재 감지 - 드론 스테이션으로 복귀 중")
                # 복귀 명령 - GUIDED 모드에서 위치 이동 (예시, 자세 제어는 별도 작성 필요)
                vehicle.mode = VehicleMode("GUIDED")
                # 예: 드론 스테이션 GPS 좌표 필요 (임의 값, 실제로 교체)
                station_gps = (37.1230, 127.5670, 0)
                # 여기에 실제 귀환 코드 삽입 (예: goto, RTL 등)
                # vehicle.simple_goto(LocationGlobalRelative(*station_gps))

                # 간단히 대기
                time.sleep(10)

                print("[INFO] 드론 스테이션 도착 - 착륙 시작")
                vehicle.mode = VehicleMode("LAND")

                # 착륙 대기 (간단 대기, 실제 착륙 완료 이벤트 기다리기 권장)
                time.sleep(15)
                print("[INFO] 착륙 완료")

                # 착륙 후 자동 충전 및 소화탄 자동 장착 (자리 확보)
                auto_attach_extinguisher()

                # 착륙 완료 및 장착 완료 알림
                print("[INFO] 착륙 및 소화탄 장착 완료. 관제 승인 대기 중...")

                # 관제 승인 플래그 ON
                continue_fire_drop = True

            # 소화탄 투하 반복 승인 및 동작
            if continue_fire_drop:
                # 관제 승인 확인 (실제는 통신 또는 UI 입력으로 변경)
                # 여기서는 단순 콘솔 입력으로 시뮬레이션
                approval = input("[입력] 소화탄 투하 승인 (y/n)? ").strip().lower()

                if approval == 'y':
                    print("[INFO] 소화탄 투하 승인 확인")

                    # 비행 모드 GUIDED로 변경 및 목표 화재 지점으로 이동
                    vehicle.mode = VehicleMode("GUIDED")

                    # 실제로 화재 지점 이동 코드 필요 (예: vehicle.simple_goto(fire_gps))

                    print(f"[INFO] 화재 지점 이동 중: {fire_gps}")
                    time.sleep(10)

                    # 소화탄 투하 동작
                    drop_extinguisher(servo)

                    # 투하 후 스크린샷 재촬영 및 저장 (선택사항)
                    ret, frame = cap.read()
                    if ret:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        filename = f"drop_{timestamp}.jpg"
                        filepath = os.path.join(IMG_SAVE_DIR, filename)
                        cv2.imwrite(filepath, frame)
                        print(f"[INFO] 투하 후 이미지 저장: {filepath}")

                    # 드론 스테이션으로 복귀 (예: simple_goto 또는 RTL)
                    print("[INFO] 드론 스테이션으로 복귀 중...")
                    time.sleep(10)

                    print("[INFO] 복귀 완료. 소화탄 투하 완료.")

                    # 투하 완료 알림 및 다음 승인 대기
                    print("[INFO] [투하 완료] 관제 승인 재확인 대기 중...")

                elif approval == 'n':
                    print("[INFO] 소화탄 투하 거부. 순찰 종료.")
                    break

                else:
                    print("[WARN] 잘못된 입력입니다. y 또는 n 입력해주세요.")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[INFO] 프로그램 종료 중...")

    finally:
        servo.cleanup()
        cap.release()
        vehicle.close()
        print("[INFO] 종료 완료")

if __name__ == "__main__":
    main()
