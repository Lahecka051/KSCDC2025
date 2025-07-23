import math
import time
import requests
import cv2
from ultralytics import YOLO
from dronekit import connect, VehicleMode, LocationGlobalRelative

# 1. Flight Controller 연결 (유선 MAVLink)
print("드론 연결 중...")
vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)

# 2. 드론 이륙 함수
def arm_and_takeoff(target_altitude):
    print("GUIDED 모드 설정 중...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)

    print("ARM 중...")
    vehicle.armed = True
    while not vehicle.armed:
        print("ARM 대기 중...")
        time.sleep(1)

    print(f"{target_altitude}m 이륙 중...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"고도: {alt:.2f}m")
        if alt >= target_altitude * 0.95:
            print("이륙 완료.")
            break
        time.sleep(1)

# 3. 위도/경도 오프셋 계산 함수
def get_offset_gps(lat, lon, heading_deg, distance_m, angle_offset_deg):
    R = 6378137.0  # Earth radius (meters)
    heading_rad = math.radians(heading_deg + angle_offset_deg)
    d_lat = (distance_m * math.cos(heading_rad)) / R
    d_lon = (distance_m * math.sin(heading_rad)) / (R * math.cos(math.radians(lat)))
    new_lat = lat + math.degrees(d_lat)
    new_lon = lon + math.degrees(d_lon)
    return new_lat, new_lon

# 4. YOLO 모델 로드
model = YOLO("/home/jetson/best.pt")
print("YOLO 모델 로딩 완료")

# 5. 카메라 초기화
cap = cv2.VideoCapture(0)  # /dev/video0
if not cap.isOpened():
    raise IOError("카메라를 열 수 없습니다.")

# 6. 드론 이륙
arm_and_takeoff(10)

# 7. 순찰 및 감지 루프
print("순찰 시작...")
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    results = model(frame)

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            name = model.names[cls]

            if name == "fire" and conf > 0.6:
                print("🔥 화재 감지!")

                # 현재 위치 및 방향
                lat = vehicle.location.global_frame.lat
                lon = vehicle.location.global_frame.lon
                heading = vehicle.heading  # 0~360도 (북쪽 기준)

                # 전방 10m 지점 계산 (정면 기준)
                fire_lat, fire_lon = get_offset_gps(lat, lon, heading, 10, 0)

                print(f"📍 추정 화재 위치: {fire_lat}, {fire_lon}")

                # 그 지점으로 이동
                vehicle.simple_goto(LocationGlobalRelative(fire_lat, fire_lon, 10))

                # 관제센터에 보고
                data = {
                    "fire_lat": fire_lat,
                    "fire_lon": fire_lon,
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "drone_id": "DRONE01"
                }
                try:
                    res = requests.post("http://192.168.0.50:5000/report_fire", json=data)
                    print(f"📡 관제센터 보고 완료: {res.status_code}")
                except Exception as e:
                    print("⚠️ 관제센터 보고 실패:", e)

                time.sleep(10)  # 중복 감지 방지

    # 필요 시 ESC로 루프 종료 가능 (테스트 중)
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 종료 처리
cap.release()
vehicle.close()
