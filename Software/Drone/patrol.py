import socket
import json
import time
import cv2
import queue
import threading
import math

# 흐름도 : 순찰 → IF_OPTIONS(화재 발견 → 사진 촬영) → 귀환 → IF_OPTIONS(화재 이미지 전송)

# [최적화] 경로점 도착 및 순찰 타임아웃 상수 추출
# 기존: 매직 넘버(2)가 코드 곳곳에 하드코딩
# 수정: 파일 상단에 상수로 정의하여 한 곳에서 관리
ARRIVAL_RADIUS = 2       # 경로점 도착 판정 반경 (미터)
WAYPOINT_TIMEOUT = 120   # 단일 경로점 이동 최대 시간 (초)

class Patrol():
    def __init__(self, drone_system, fire_detector, landing, communicator):
        self.drone_system = drone_system
        self.fire_detector = fire_detector
        self.landing = landing
        self.communicator = communicator
        self.status_q = queue.Queue()
        self.stop_event = threading.Event()

        # fire_detector에 drone_system 참조 주입
        self.fire_detector.drone_system = drone_system

    def run(self, path):
        if not path:
            return

        fire_confirmed = False
        align = False
        coordinates = None
        image_path = None
        self.stop_event.clear()
        status = "MOVING_TO_TARGET"

        drone_gps = {
            'lat': 0,
            'lon': 0,
            'alt': 2.0,
            'heading': 0
        }

        # 화재 감지 스레드 시작
        threading.Thread(
            target=self.fire_detector.fire_detection_thread,
            args=(drone_gps, self.status_q, self.stop_event),
            daemon=True
        ).start()

        self.drone_system.arm()
        time.sleep(2)
        self.drone_system.takeoff()
        time.sleep(5)

        for i, waypoint in enumerate(path):
            lat = waypoint['lat']
            lon = waypoint['lon']
            print(f"[순찰] 경로점 {i+1} ({lat:.7f},{lon:.7f}) 이동 시작")

            # [최적화] 경로점 이동 루프에 타임아웃 추가
            # 기존: while not fire_confirmed: (무한 루프 — GPS 장애 시 탈출 불가)
            # 수정: 시작 시간 기록 후 WAYPOINT_TIMEOUT 초과 시 다음 경로점으로 강제 이동
            wp_start_time = time.time()

            while not fire_confirmed:
                # [최적화] 타임아웃 체크
                if time.time() - wp_start_time > WAYPOINT_TIMEOUT:
                    print(f"[순찰] 경로점 {i+1} 타임아웃 ({WAYPOINT_TIMEOUT}초) - 다음 경로점으로 이동")
                    break

                try:
                    result = self.status_q.get_nowait()
                    if result["status"] == "started":
                        status = "OBSERVING"

                    elif result["status"] == "recognized":
                        status = "FIRE_DETECTED"
                        fire_coords = result["coords"]

                    elif result["status"] == "failed":
                        status = "MOVING_TO_TARGET"

                except queue.Empty:
                    pass

                if status == "OBSERVING":
                    print("[순찰] 호버링, 화재 관찰중")
                elif status == "FIRE_DETECTED":
                    fire_confirmed = True
                    self.stop_event.set()
                    print("[순찰] 화재 감지, 화재 지점으로 이동합니다")
                    break
                elif status == "MOVING_TO_TARGET":
                    self.drone_system.goto_gps(lat, lon)
                    current_coord = self.drone_system.read_gps()

                    if current_coord:
                        current_lat = current_coord["lat"]
                        current_lon = current_coord["lon"]

                        # drone_gps 업데이트 (화재 감지 스레드와 공유)
                        drone_gps['lat'] = current_lat
                        drone_gps['lon'] = current_lon

                        distance = self.drone_system.get_distance_metres(
                            current_lat, current_lon, lat, lon
                        )

                        if distance <= ARRIVAL_RADIUS:
                            print(f"[순찰] waypoint {i+1} 도착")
                            break
                    else:
                        print("[순찰] GPS 읽기 실패, 재시도...")
                        time.sleep(0.5)

        if fire_confirmed:
            fire_lat = fire_coords[0]
            fire_lon = fire_coords[1]

            # [최적화] 화재 지점 이동 루프에도 타임아웃 추가
            # 기존: while True: (무한 루프)
            # 수정: 60초 타임아웃 추가
            fire_move_start = time.time()
            FIRE_MOVE_TIMEOUT = 60

            while time.time() - fire_move_start < FIRE_MOVE_TIMEOUT:
                self.drone_system.goto_gps(fire_lat, fire_lon)

                current_coord = self.drone_system.read_gps()
                if current_coord:
                    current_lat = current_coord["lat"]
                    current_lon = current_coord["lon"]

                    distance_fire = self.drone_system.get_distance_metres(
                        current_lat, current_lon, fire_lat, fire_lon
                    )

                    if distance_fire <= ARRIVAL_RADIUS:
                        print(f"[순찰] 화재 지점 근처 도착")
                        break

                time.sleep(0.1)

            # 객체 정렬
            while not align:
                align, cmd = self.fire_detector.align_drone_to_object()
                self.drone_system.set_command(cmd)
                if self.fire_detector.patrol_state == 'stop':
                    break

            if not self.fire_detector.patrol_state == 'stop':
                coordinates = self.drone_system.read_gps()
                image_path = self.fire_detector.capture_and_save_image()

        # 홈으로 복귀
        self.drone_system.goto_home()
        self.landing.run()

        # 화재 보고 전송
        if fire_confirmed and coordinates and image_path:
            self.communicator.send_fire_report(coordinates, image_path)
