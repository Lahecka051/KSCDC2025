# Fire_detect.py
import cv2
import numpy as np
import time
import math
from ultralytics import YOLO
import threading


class Fire_detector:
    def __init__(self, cap0, cap1):
        self.cap0 = cap0
        self.cap1 = cap1
        self.model = YOLO("best.engine")
        self.frame_width0 = int(self.cap0.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height0 = int(self.cap0.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_width1 = int(self.cap1.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height1 = int(self.cap1.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.center_frame_x1 = self.frame_width1 // 2
        self.center_frame_y1 = self.frame_height1 // 2
        self.class_names = self.model.names

        # --- fire gps에 사용되는 변수 ---
        self.FIRE_CONFIRMATION = 10  # 화재 판정을 위한 연속 감지 기준
        self.OBSERVATION_TIMEOUT = 5
        self.CAMERA_TILT_DEGREES = 30.0 

        # --- 드론 중앙정렬에 쓰이는 변수 ---
        self.Kp_x = 0.05
        self.Kp_y = 0.05
        self.threshold = 10

        # 순찰 관련 변수
        self.patrol_mode = True
        self.patrol_state = 'top'
        self.patrol_laps = 0
        self.max_patrol_laps = 2

        self.target_classes = ['fire', 'smoke']


    def fire_gps(self, drone_gps, center_x, center_y, frame_width=1280, frame_height=1280, camera_fov_h=65.0):
        """
        정면을 바라보는 카메라를 기준으로 화재 지점의 GPS 좌표를 추정합니다.
        드론이 수평 상태(Roll=0, Pitch=0)라고 가정합니다.
        
        :param drone_gps: 드론의 현재 GPS 및 상태 정보 (위도, 경도, 상대고도, 헤딩)
        :param center_x: 프레임에서 감지된 화재의 중심 x좌표
        :param center_y: 프레임에서 감지된 화재의 중심 y좌표
        :param frame_width: 카메라 프레임의 너비
        :param frame_height: 카메라 프레임의 높이
        :param camera_fov_h: 카메라의 수평 화각(Field of View) (도 단위) 65도
        """
        
        # --- 1. 픽셀 좌표를 카메라 중심 기준 각도로 변환 ---
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2

        # 픽셀당 각도 계산 (화각 이용)
        pixels_per_degree_h = frame_width / camera_fov_h
        
        # 카메라 중심으로부터의 좌우 각도 (Azimuth)
        angle_x_from_center = (center_x - frame_center_x) / pixels_per_degree_h
        
        # 카메라 중심으로부터의 상하 각도 (Elevation)
        angle_y_from_center = (center_y - frame_center_y) / pixels_per_degree_h

        # --- 2. 최종 지면 각도 계산 ---
        # 드론의 헤딩 방향을 기준으로, 화재 지점까지의 최종 좌우 각도
        total_angle_azimuth = drone_gps.heading + angle_x_from_center
        
        # 드론의 수평선을 기준으로, 화재 지점까지의 최종 하방 각도 (고개 숙인 각도)
        total_angle_depression = self.CAMERA_TILT_DEGREES + angle_y_from_center

        # 라디안으로 변환
        total_angle_azimuth_rad = math.radians(total_angle_azimuth)
        total_angle_depression_rad = math.radians(total_angle_depression)

        # --- 3. 삼각함수를 이용해 거리 계산 ---
        # 고도와 하방 각도를 이용해 드론과 화재 지점 사이의 수평 거리 계산
        horizontal_distance = drone_gps.altitude / math.tan(total_angle_depression_rad)

        # --- 4. 거리와 방향을 이용해 GPS 좌표 변화량 계산 ---
        earth_radius = 6371000 # 지구 반지름 (미터)

        # 위도 변화량 계산 (북-남 방향 거리)
        delta_lat = (horizontal_distance * math.cos(total_angle_azimuth_rad)) / earth_radius
        
        # 경도 변화량 계산 (동-서 방향 거리)
        delta_lon = (horizontal_distance * math.sin(total_angle_azimuth_rad)) / \
                    (earth_radius * math.cos(math.radians(drone_gps.latitude)))

        # --- 5. 최종 GPS 좌표 계산 ---
        estimated_lat = drone_gps.latitude + math.degrees(delta_lat)
        estimated_lon = drone_gps.longitude + math.degrees(delta_lon)

        return (estimated_lat, estimated_lon)

    def fire_detection_thread(self, drone_gps, result_q, stop_event):
        """
        스레드에서 실행되는 화재 감지 루프
        - 감지 시작시: {"status": "started"}
        - 5초 연속 감지 완료시: {"status": "recognized", "coords": (lat, lon)}
        - 중간에 끊기면: {"status": "failed"}
        """
        detecting = False
        start_time = None
        detection_count = 0
        last_coords = None

        while not stop_event.is_set():
            ret, frame = self.cap0.read()
            if not ret:
                continue

            results = self.model.predict(frame, imgsz=920, conf=0.4, verbose=False)

            fire_detected = False
            center_x, center_y = None, None

            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda box: box.conf[0])
                class_name = self.class_names[int(best_box.cls[0])]
                if class_name.lower() in ["fire", "smoke"]:
                    fire_detected = True
                    x1, y1, x2, y2 = best_box.xyxy[0]
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

            if fire_detected:
                coords = self.fire_gps(drone_gps, center_x, center_y)
                last_coords = coords

                if not detecting:
                    detecting = True
                    start_time = time.time()
                    detection_count = 1
                    result_q.put({"status": "started"})   # 감지 시작 알림
                else:
                    detection_count += 1

                elapsed = time.time() - start_time
                if elapsed >= self.OBSERVATION_TIMEOUT:  # 5초 관찰
                    if detection_count >= self.FIRE_CONFIRMATION:  # 최소 감지 횟수 조건
                        result_q.put({"status": "recognized", "coords": last_coords})
                    else:
                        result_q.put({"status": "failed"})
                    detecting = False
                    detection_count = 0
                    start_time = None

            else:
                if detecting:
                    # 감지 중이었는데 끊김 → 실패 판정
                    result_q.put({"status": "failed"})
                    detecting = False
                    detection_count = 0
                    start_time = None

    def patrol_logic(self):
        if self.patrol_laps >= self.max_patrol_laps:
            print("최대 순찰 횟수 도달. 순찰 종료 및 호버링.")
            self.patrol_mode = False
            self.patrol_state = 'stop'
            self.patrol_laps = 0
            # DroneController 형식으로 변경 [전진/후진, 좌/우, 상승/하강, 방향]
            return [0, 0, 0, 0]  # 호버링

        if self.patrol_state == 'top':
            # 전진 1m/s
            cmd = [1, 0, 0, 0]
            self.patrol_state = 'right'
        elif self.patrol_state == 'right':
            # 우측 1m/s
            cmd = [0, -1, 0, 0]
            self.patrol_state = 'bottom'
        elif self.patrol_state == 'bottom':
            # 후진 1m/s
            cmd = [-1, 0, 0, 0]
            self.patrol_state = 'left'
        elif self.patrol_state == 'left':
            # 좌측 1m/s
            cmd = [0, 1, 0, 0]
            self.patrol_state = 'top'
            self.patrol_laps += 1
        return cmd

    def align_drone_to_object(self):
        ret, frame = self.cap1.read()
        if not ret:
            print("하단 카메라 프레임을 읽을 수 없습니다.")
            # DroneController 형식
            return False, [0, 0, 0, 0]

        results = self.model.predict(frame, imgsz=960, conf=0.4, verbose=False)

        is_target_detected = False
        best_box = None
        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            if class_name.lower() in self.target_classes:
                is_target_detected = True

        if self.patrol_mode:
            if is_target_detected:
                print(f"특정 객체 '{class_name}' 감지 성공. 정렬 모드로 전환.")
                self.patrol_mode = False
            else:
                cmd = self.patrol_logic()
                time.sleep(5)
                return False, cmd

        if not self.patrol_mode:
            if not is_target_detected:
                print("정렬 중 객체 탐지 실패. 순찰 모드로 전환.")
                self.patrol_mode = True
                self.patrol_state = 'top'
                self.patrol_laps = 0
                self.last_move_time = time.time()
                cmd = self.patrol_logic()
                return False, cmd

            x1, y1, x2, y2 = best_box.xyxy[0]
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            error_x = center_x - self.center_frame_x1
            error_y = center_y - self.center_frame_y1

            if abs(error_x) < self.threshold and abs(error_y) < self.threshold:
                print(f"✅ 정렬 완료! 오차: ({error_x}, {error_y})")
                self.patrol_mode = True
                self.patrol_laps = 0
                # 호버링
                return True, [0, 0, 0, 0]

            # 속도 계산 (m/s 단위로 변환)
            speed_x = abs(error_x) * self.Kp_x / 100  # 픽셀을 m/s로 변환
            speed_y = abs(error_y) * self.Kp_y / 100

            speed_x = max(0.1, min(0.8, speed_x))
            speed_y = max(0.1, min(0.8, speed_y))

            # DroneController 형식으로 명령 생성
            vx = 0  # 전진/후진
            vy = 0  # 좌/우
            
            if abs(error_y) > self.threshold:
                vx = speed_y if error_y > 0 else -speed_y
            
            if abs(error_x) > self.threshold:
                vy = -speed_x if error_x > 0 else speed_x  # 좌측이 +이므로
                
            print(f"오차: ({error_x}, {error_y}), 속도: vx={vx:.2f}, vy={vy:.2f}")
            return False, [vx, vy, 0, 0]

    def capture_and_save_image(self, output_path="captured_image.jpg"):
        """
        CSI 카메라(하단 카메라)로 사진을 찍어 지정된 경로에 저장합니다.
        """
        if not self.cap1.isOpened():
            print("오류: 카메라를 열 수 없습니다. 카메라 연결 및 파이프라인 설정을 확인해주세요.")
            return

        ret, frame = self.cap1.read()
        if ret:
            cv2.imwrite(output_path, frame)
            print(f"사진이 성공적으로 촬영되어 {output_path}에 저장되었습니다.")
            # 수정: release 제거 (계속 사용해야 하므로)
            # self.cap1.release()
            return
        else:
            print("오류: 프레임을 읽을 수 없습니다.")
            return