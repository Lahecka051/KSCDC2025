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

        # fire gps 변수 (카메라 Frame 640x640, FOV 63도 기준 30fps)
        self.FIRE_CONFIRMATION = 30  # 화재 확정 프레임 임계값
        self.OBSERVATION_TIMEOUT = 5  # 화재 탐지 관찰 시간
        self.CAMERA_TILT_DEGREES = 30.0  # 카메라 각도
        self.MAX_FIRE_DISTANCE = 20  # 최대 화재 거리

        # 정렬 판정 임계값
        self.threshold = 30
        
        # 단순 이동 속도
        self.MOVE_SPEED = 0.1

        # 순찰 관련 변수
        self.patrol_mode = True
        self.patrol_state = 'top'
        self.patrol_laps = 0
        self.max_patrol_laps = 2

        self.target_classes = ['fire', 'smoke']

    def fire_gps(self, drone_gps, center_x, center_y, frame_width=640, frame_height=640, camera_fov_h=63.0):
        """화재 GPS 좌표 추정"""
        # #수정: drone_gps가 딕셔너리 형태로 전달되는 경우 처리
        if drone_gps is None:
            print("[화재감지] 오류: drone_gps 객체가 None입니다")
            return None
            
        # #수정: 딕셔너리 형태로 처리
        if isinstance(drone_gps, dict):
            latitude = drone_gps.get('lat')
            longitude = drone_gps.get('lon')
            altitude = drone_gps.get('alt', 2.0)  # 기본값 2.0m
            heading = drone_gps.get('heading', 0)  # 기본값 0도
        else:
            # 객체 형태로 처리 (기존 코드와 호환)
            latitude = getattr(drone_gps, 'latitude', None) or getattr(drone_gps, 'lat', None)
            longitude = getattr(drone_gps, 'longitude', None) or getattr(drone_gps, 'lon', None)
            altitude = getattr(drone_gps, 'altitude', None) or getattr(drone_gps, 'alt', 2.0)
            heading = getattr(drone_gps, 'heading', 0)
            
        if latitude is None or longitude is None:
            print("[화재감지] 오류: GPS 좌표가 없습니다")
            return None
            
        if altitude is None or altitude < 0.5:
            print("[화재감지] 오류: 유효하지 않은 고도")
            return None
            
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2

        pixels_per_degree_h = frame_width / camera_fov_h
        
        angle_x_from_center = (center_x - frame_center_x) / pixels_per_degree_h
        angle_y_from_center = (center_y - frame_center_y) / pixels_per_degree_h

        total_angle_azimuth = heading + angle_x_from_center
        total_angle_depression = self.CAMERA_TILT_DEGREES + angle_y_from_center

        total_angle_azimuth_rad = math.radians(total_angle_azimuth)
        total_angle_depression_rad = math.radians(total_angle_depression)

        if total_angle_depression_rad > 0:
            horizontal_distance = altitude / math.tan(total_angle_depression_rad)
            
            if horizontal_distance > self.MAX_FIRE_DISTANCE:
                print(f"[화재감지] 오류: 추정 거리 {horizontal_distance:.1f}m가 최대 거리 {self.MAX_FIRE_DISTANCE}m 초과")
                return None
            elif horizontal_distance < 0:
                horizontal_distance = 10
        else:
            horizontal_distance = 10

        earth_radius = 6371000

        delta_lat = (horizontal_distance * math.cos(total_angle_azimuth_rad)) / earth_radius
        delta_lon = (horizontal_distance * math.sin(total_angle_azimuth_rad)) / \
                    (earth_radius * math.cos(math.radians(latitude)))

        estimated_lat = latitude + math.degrees(delta_lat)
        estimated_lon = longitude + math.degrees(delta_lon)

        print(f"[화재감지] 정상: 추정 거리 {horizontal_distance:.1f}m, 고도: {altitude:.1f}m")
        
        return (estimated_lat, estimated_lon)

    def fire_detection_thread(self, drone_gps, result_q, stop_event):
        """스레드에서 실행되는 화재 감지 루프"""
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
                # #수정: 실시간으로 GPS 업데이트 (딕셔너리 형태로 전달)
                if hasattr(self, 'drone_system'):
                    current_gps = self.drone_system.read_gps()
                    if current_gps:
                        current_gps['alt'] = getattr(self.drone_system, 'set_alt', 2.0)
                        current_gps['heading'] = 0  # 실제 heading 값이 필요하면 추가
                        coords = self.fire_gps(current_gps, center_x, center_y)
                else:
                    coords = self.fire_gps(drone_gps, center_x, center_y)
                
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

    def capture_cam0_image(self, output_path="fire_detection.jpg"):
        """정면 카메라(cam0)로 사진 촬영"""
        if not self.cap0.isOpened():
            print("[화재감지] 오류: 정면 카메라를 열 수 없습니다.")
            return False

        ret, frame = self.cap0.read()
        if ret:
            cv2.imwrite(output_path, frame)
            print(f"[화재감지] 화재 사진이 {output_path}에 저장되었습니다.")
            return True
        else:
            print("[화재감지] 오류: 프레임을 읽을 수 없습니다.")
            return False
    
    def capture_and_save_image(self, output_path="captured_image.jpg"):
        """CSI 카메라(하단 카메라)로 사진 촬영"""
        if not self.cap1.isOpened():
            print("[화재감지] 오류: 카메라를 열 수 없습니다.")
            return output_path  # #수정: False 대신 경로 반환

        ret, frame = self.cap1.read()
        if ret:
            cv2.imwrite(output_path, frame)
            print(f"[화재감지] 사진이 {output_path}에 저장되었습니다.")
            return output_path  # #수정: True 대신 경로 반환
        else:
            print("[화재감지] 오류: 프레임을 읽을 수 없습니다.")
            return output_path  # #수정: False 대신 경로 반환