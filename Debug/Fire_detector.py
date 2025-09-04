# Fire_detector.py
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

        # fire gps에 사용되는 변수
        self.FIRE_CONFIRMATION = 10
        self.OBSERVATION_TIMEOUT = 5
        self.CAMERA_TILT_DEGREES = 30.0
        self.MAX_FIRE_DISTANCE = 15  # 최대 화재 추정 거리 (미터)

        # 정렬 판정 임계값 (픽셀)
        self.threshold = 30
        
        # 단순 이동 속도 (m/s)
        self.MOVE_SPEED = 0.15

        # 순찰 관련 변수
        self.patrol_mode = True
        self.patrol_state = 'top'
        self.patrol_laps = 0
        self.max_patrol_laps = 2

        self.target_classes = ['fire', 'smoke']


    def fire_gps(self, drone_gps, center_x, center_y, frame_width=640, frame_height=640, camera_fov_h=65.0):
        """
        정면을 바라보는 카메라를 기준으로 화재 지점의 GPS 좌표를 추정합니다.
        최대 거리를 초과하면 None을 반환합니다.
        """
        # 수정: GPS 유효성 체크
        if drone_gps.latitude is None or drone_gps.longitude is None:
            print("[화재감지] 오류: GPS 좌표가 없습니다")
            return None
            
        # 픽셀 좌표를 카메라 중심 기준 각도로 변환
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2

        # 픽셀당 각도 계산
        pixels_per_degree_h = frame_width / camera_fov_h
        
        # 카메라 중심으로부터의 좌우 각도
        angle_x_from_center = (center_x - frame_center_x) / pixels_per_degree_h
        
        # 카메라 중심으로부터의 상하 각도
        angle_y_from_center = (center_y - frame_center_y) / pixels_per_degree_h

        # 최종 지면 각도 계산
        total_angle_azimuth = drone_gps.heading + angle_x_from_center
        total_angle_depression = self.CAMERA_TILT_DEGREES + angle_y_from_center

        # 라디안으로 변환
        total_angle_azimuth_rad = math.radians(total_angle_azimuth)
        total_angle_depression_rad = math.radians(total_angle_depression)

        # 삼각함수를 이용해 거리 계산
        if total_angle_depression_rad > 0:
            horizontal_distance = drone_gps.altitude / math.tan(total_angle_depression_rad)
            
            # 수정: 최대 거리 초과 시 None 반환 (오류 판정)
            if horizontal_distance > self.MAX_FIRE_DISTANCE:
                print(f"[화재감지] 오류: 추정 거리 {horizontal_distance:.1f}m가 최대 거리 {self.MAX_FIRE_DISTANCE}m 초과")
                return None  # None 반환하여 오류 표시
            elif horizontal_distance < 0:
                horizontal_distance = 10  # 최소 거리
        else:
            horizontal_distance = 10  # 기본값

        # 거리와 방향을 이용해 GPS 좌표 변화량 계산
        earth_radius = 6371000  # 지구 반지름 (미터)

        # 위도 변화량 계산
        delta_lat = (horizontal_distance * math.cos(total_angle_azimuth_rad)) / earth_radius
        
        # 경도 변화량 계산
        delta_lon = (horizontal_distance * math.sin(total_angle_azimuth_rad)) / \
                    (earth_radius * math.cos(math.radians(drone_gps.latitude)))

        # 최종 GPS 좌표 계산
        estimated_lat = drone_gps.latitude + math.degrees(delta_lat)
        estimated_lon = drone_gps.longitude + math.degrees(delta_lon)

        print(f"[화재감지] 정상: 추정 거리 {horizontal_distance:.1f}m, 고도: {drone_gps.altitude:.1f}m")
        
        return (estimated_lat, estimated_lon)

    def fire_detection_thread(self, drone_gps, result_q, stop_event):
        """
        스레드에서 실행되는 화재 감지 루프
        """
        detecting = False
        start_time = None
        detection_count = 0
        last_coords = None

        while not stop_event.is_set():
            ret, frame = self.cap0.read()
            if not ret:
                continue

            results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)

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
                
                # 수정: fire_gps가 None 반환 시 오류 처리
                if coords is None:
                    result_q.put({"status": "error", "message": "화재 거리 계산 오류"})
                    detecting = False
                    continue
                    
                last_coords = coords

                if not detecting:
                    detecting = True
                    start_time = time.time()
                    detection_count = 1
                    result_q.put({"status": "started"})
                else:
                    detection_count += 1

                elapsed = time.time() - start_time
                if elapsed >= self.OBSERVATION_TIMEOUT:
                    if detection_count >= self.FIRE_CONFIRMATION:
                        result_q.put({"status": "recognized", "coords": last_coords})
                    else:
                        result_q.put({"status": "failed"})
                    detecting = False
                    detection_count = 0
                    start_time = None

            else:
                if detecting:
                    result_q.put({"status": "failed"})
                    detecting = False
                    detection_count = 0
                    start_time = None

    # 나머지 메서드는 동일...
    def patrol_logic(self):
        """사각형 패턴으로 순찰하는 로직"""
        if self.patrol_laps >= self.max_patrol_laps:
            print("[화재감지] 최대 순찰 횟수 도달. 순찰 종료 및 호버링.")
            self.patrol_mode = False
            self.patrol_state = 'stop'
            self.patrol_laps = 0
            return [0, 0, 0, 0]

        PATROL_SPEED = 0.5
        
        if self.patrol_state == 'top':
            cmd = [PATROL_SPEED, 0, 0, 0]
            self.patrol_state = 'right'
        elif self.patrol_state == 'right':
            cmd = [0, -PATROL_SPEED, 0, 0]
            self.patrol_state = 'bottom'
        elif self.patrol_state == 'bottom':
            cmd = [-PATROL_SPEED, 0, 0, 0]
            self.patrol_state = 'left'
        elif self.patrol_state == 'left':
            cmd = [0, PATROL_SPEED, 0, 0]
            self.patrol_state = 'top'
            self.patrol_laps += 1
        else:
            cmd = [0, 0, 0, 0]
            
        return cmd

    def align_drone_to_object(self):
        """하단 카메라를 사용해 드론을 화재 위에 정렬"""
        ret, frame = self.cap1.read()
        if not ret:
            print("[화재감지] 하단 카메라 프레임을 읽을 수 없습니다.")
            return False, [0, 0, 0, 0]

        results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)

        is_target_detected = False
        best_box = None
        class_name = ""
        
        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            if class_name.lower() in self.target_classes:
                is_target_detected = True

        if self.patrol_mode:
            if is_target_detected:
                print(f"[화재감지] 객체 '{class_name}' 감지. 정렬 시작.")
                self.patrol_mode = False
            else:
                cmd = self.patrol_logic()
                time.sleep(5)
                return False, cmd

        if not self.patrol_mode:
            if not is_target_detected:
                print("[화재감지] 정렬 중 객체 탐지 실패. 순찰 모드로 전환.")
                self.patrol_mode = True
                self.patrol_state = 'top'
                self.patrol_laps = 0
                cmd = self.patrol_logic()
                return False, cmd

            x1, y1, x2, y2 = best_box.xyxy[0]
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            error_x = center_x - self.center_frame_x1
            error_y = center_y - self.center_frame_y1

            if abs(error_x) < self.threshold and abs(error_y) < self.threshold:
                print(f"[화재감지] 정렬 완료! 오차: ({error_x}, {error_y})")
                self.patrol_mode = True
                self.patrol_laps = 0
                return True, [0, 0, 0, 0]

            vx = 0
            vy = 0
            
            if error_y > self.threshold:
                vx = self.MOVE_SPEED
            elif error_y < -self.threshold:
                vx = -self.MOVE_SPEED
            
            if error_x > self.threshold:
                vy = -self.MOVE_SPEED
            elif error_x < -self.threshold:
                vy = self.MOVE_SPEED
                
            print(f"[화재감지] 정렬 중... 오차: ({error_x}, {error_y}), 명령: vx={vx:.2f}, vy={vy:.2f}")
            return False, [vx, vy, 0, 0]

    def capture_and_save_image(self, output_path="captured_image.jpg"):
        """CSI 카메라(하단 카메라)로 사진 촬영"""
        if not self.cap1.isOpened():
            print("[화재감지] 오류: 카메라를 열 수 없습니다.")
            return False

        ret, frame = self.cap1.read()
        if ret:
            cv2.imwrite(output_path, frame)
            print(f"[화재감지] 사진이 {output_path}에 저장되었습니다.")
            return True
        else:
            print("[화재감지] 오류: 프레임을 읽을 수 없습니다.")
            return False