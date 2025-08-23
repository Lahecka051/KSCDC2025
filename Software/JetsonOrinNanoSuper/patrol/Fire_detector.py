#Fire_detect.py
import cv2
import numpy as np
import time
from ultralytics import YOLO

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
    self.observing = False
    self.start_time = None
    self.last_detection = None
    self.detection_count = 0
    self.FIRE_CONFIRMATION_DURATION = 3 #화제 판정을 위한 연속 감지 시간
    self.OBSERVATIOPN_TIMEOUT = 5

    # --- 드론 중앙정렬에 쓰이는 변수 ---
    #제어 상수 (튜닝 필요)
    self.Kp_x = 0.05 
    self.Kp_y = 0.05
    self.threshold = 10
    # 순찰 관련 변수
    self.patrol_mode = True
    self.patrol_state = 'top'
    self.last_move_time = time.time()
    self.move_duration = 3  # 각 방향으로 이동하는 시간 (초)
    self.is_moving = False
    self.patrol_laps = 0
    self.max_patrol_laps = 2 # 최대 순찰 바퀴 수

     # 화재 지점 GPS 추정 함수
    def fire_gps(self, drone_gps, center_x, center_y):               
      # 화재 지점의 각도 계산 (가정: 카메라 시야각 90도, 프레임 중심에서 픽셀당 각도 계산)
      frame_center_x, frame_center_y = self.frame_width0 // 2, self.frame_height0 // 2
      pixels_per_degree = self.frame_width0 / 90.0
      angle_x = (center_x - frame_center_x) / pixels_per_degree
      angle_y = (center_y - frame_center_y) / pixels_per_degree
        # 이 함수는 드론의 현재 GPS, Yaw, 피치, 롤, 고도, 그리고 카메라 시야각을
        # 기반으로 지상의 화재 지점 GPS를 추정합니다.
        # 실제 구현은 복잡하며, 여기서는 단순화된 가정을 사용합니다.
        
        # 가정: angle_x는 좌우 회전각, angle_y는 상하 기울기각 (피치)
        # GPS 이동은 단순화된 GPS 좌표계에서 계산됩니다.
      earth_radius = 6371000  # 지구 반지름 (미터)
        
        # 각도를 라디안으로 변환
      angle_x_rad = math.radians(angle_x)
      angle_y_rad = math.radians(angle_y)
      heading_rad = math.radians(drone_gps.heading)
        
        # 고도와 각도를 이용해 지상과의 수평 거리 계산
        # 드론 자세(피치, 롤)를 고려해야 하지만, 여기서는 단순화를 위해 생략
      horizontal_distance = drone_gps.altitude * math.tan(angle_y_rad)
        
      # 새로운 GPS 좌표 계산 (간단한 평면 지구 모델 가정)
      delta_lat = (horizontal_distance * math.cos(heading_rad + angle_x_rad)) / earth_radius
      delta_lon = (horizontal_distance * math.sin(heading_rad + angle_x_rad)) / (earth_radius * math.cos(math.radians(drone_gps.latitude)))
        
      estimated_lat = drone_gps.latitude + math.degrees(delta_lat)
      estimated_lon = drone_gps.longitude + math.degrees(delta_lon)

      return (estimated_lat, estimated_lon)

    def detect_fire_upper(self,drone_gps):
      ret, frame = self.cap0.read()
      if not ret: return False, None, None
      results = self.model.predict(frame, imgsz=920, conf=0.4, verbose=False)
      fire_detected = False
      best_box = None
      center_x = None
      center_y = None
      if len(results[0].boxes) > 0:
        best_box = max(results[0].boxes, key=lambda box: box.conf[0])
        class_name = self.class_names[int(best_box.cls[0])]   
        if class_name.lower() in ["fire", "smoke"]:
          fire_detected = True
          x1,y1,x2,y2 = best_box.xyxy[0]
          center_x = int((x1+x2)/2)
          center_y = int((y1+y2)/2)

      fire_confirmed = False
      fire_coords = None
      if fire_detectde:
        if not self.observing:
          self.observing = True
          self.start_time = time.time()
          self.detection_count = 1
        else:
          self.detection_count += 1

        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.OBSERVATION_TIMEOUT:
          if self.detection_count >= 3:
            fire_confirmed = True
            fire_coords = self.fire_gps(drone_gps, center_x, center_y)
          self.observing = False
          self.detection_count = 0
          self.start_time = None
      
      else:
        if self.observing:
          self.observig = False
          self.detection_count = 0
          self.start_time = None

      return fire_confirmed, fire_coords
  
    def patrol_logic(self):
        """
        사각형 순찰 로직을 수행하고, 드론 제어 명령을 반환합니다.
        순찰이 완료되면 False를 반환합니다.
        """
        if self.patrol_laps >= self.max_patrol_laps:
            print("최대 순찰 횟수 도달. 순찰 종료 및 호버링.")
            self.patrol_mode = False
            return ["level", "hover", 0, 0]

        if time.time() - self.last_move_time > self.move_duration:
            if self.patrol_state == 'top':
                print("순찰: 전진")
                cmd = ["level", "forward", 0, 30]
                self.patrol_state = 'right'
            elif self.patrol_state == 'right':
                print("순찰: 우측 회전")
                cmd = ["level", "hover", 90, 0]
                self.patrol_state = 'bottom'
            elif self.patrol_state == 'bottom':
                print("순찰: 후진")
                cmd = ["level", "backward", 0, 30]
                self.patrol_state = 'left'
            elif self.patrol_state == 'left':
                print("순찰: 좌측 회전")
                cmd = ["level", "hover", -90, 0]
                self.patrol_state = 'top'
                self.patrol_laps += 1 # 한 바퀴 완료
            self.last_move_time = time.time()
            self.is_moving = True
            return cmd
        else:
            if not self.is_moving:
                cmd = ["level", "hover", 0, 0]
                self.is_moving = True
                return cmd
        return None # 명령이 없을 경우

    def align_drone_to_object(self):
        """
        하단 카메라에 잡힌 객체 중심에 드론을 일치시키는 메서드.
        단일 프레임을 처리하고 명령어를 반환합니다.

        Returns:
            tuple: (정렬 완료 여부, 드론 제어 명령어)
        """
        ret, frame = self.cap1.read()
        if not ret:
            print("하단 카메라 프레임을 읽을 수 없습니다.")
            return (False, ["level", "hover", 0, 0])

        results = self.model.predict(frame, imgsz=640, conf=0.4, verbose=False)
        
        is_target_detected = False
        best_box = None
        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            if class_name.lower() in self.target_classes:
                is_target_detected = True

        # 순찰 모드일 때
        if self.patrol_mode:
            if is_target_detected:
                print(f"특정 객체 '{class_name}' 감지 성공. 정렬 모드로 전환.")
                self.patrol_mode = False
            else:
                # 객체가 없으면 순찰 로직 실행
                cmd = self.patrol_logic()
                return (False, cmd)

        # 정렬 모드일 때
        if not self.patrol_mode:
            if not is_target_detected:
                print("정렬 중 객체 탐지 실패. 순찰 모드로 전환.")
                self.patrol_mode = True
                self.patrol_state = 'top'
                self.patrol_laps = 0
                self.last_move_time = time.time()
                cmd = self.patrol_logic()
                return (False, cmd)
            
            # 정렬 로직
            x1, y1, x2, y2 = best_box.xyxy[0]
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            error_x = center_x - self.center_frame_x
            error_y = center_y - self.center_frame_y
            
            if abs(error_x) < self.threshold and abs(error_y) < self.threshold:
                print(f"✅ 정렬 완료! 오차: ({error_x}, {error_y})")
                self.patrol_mode = True
                self.patrol_laps = 0
                return (True, ["level", "hover", 0, 0])
            
            speed_x = int(abs(error_x) * self.Kp_x)
            speed_y = int(abs(error_y) * self.Kp_y)
            
            speed_x = max(10, min(80, speed_x))
            speed_y = max(10, min(80, speed_y))
            #===================수정이 필요한 부분======================
            # 오차가 작으면 이동하지 않도록 로직을 간소화
            if abs(error_x) <= self.threshold:
                horizontal_cmd = "hover"
            else:
                horizontal_cmd = "right" if error_x > 0 else "left"
            
            if abs(error_y) <= self.threshold:
                vertical_cmd = "hover"
            else:
                vertical_cmd = "forward" if error_y > 0 else "backward"

            # 호버링 명령은 이미 정렬 완료 조건에서 처리되었으므로,
            # 여기서는 이동 명령만 반환합니다.
            print(f"오차: ({error_x}, {error_y}), 명령: {vertical_cmd}, {horizontal_cmd}")
            if horizontal_cmd == "hover" and vertical_cmd != "hover":
                return (False, ["level", vertical_cmd, 0, speed_y])
            elif horizontal_cmd != "hover" and vertical_cmd == "hover":
                return (False, ["level", horizontal_cmd, 0, speed_x])
            else:
                return (False, ["level", f"{vertical_cmd}_{horizontal_cmd}", 0, max(speed_x, speed_y)])
          #======================================================
        else:
            print("특정 객체 탐지 실패. 순찰 모드로 유지.")
            self.patrol_mode = True
            cmd = self.patrol_logic()
            return (False, cmd)
