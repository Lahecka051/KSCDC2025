#Fire_detect.py
import cv2
import numpy as np
import time
from ultralytics import YOLO

class Fire_detect:
  def __init__(self, cap0, cap1):
    self.cap0 = cap0
    self.cap1 = cap1
    self.model = YOLO("best.engine")
    self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    self.w1, self.w2 = self.frame_width // 3, 2 * self.frame_width // 3
    self.h1, self.h2 = self.frame_height // 3, 2 * self.frame_height // 3
    self.class_names = self.model.names

  def get_position_command(self, x, y):
        if y < self.h1: vertical = "위"
        elif y < self.h2: vertical = "가운데"
        else: vertical = "아래"
        if x < self.w1: horizontal = "왼쪽"
        elif x < self.w2: horizontal = "가운데"
        else: horizontal = "오른쪽"
        mapping = {
            ("위","왼쪽"): ["level","forward_left",0,10],
            ("위","가운데"): ["level","forward",0,10],
            ("위","오른쪽"): ["level","forward_right",0,10],
            ("가운데","왼쪽"): ["level","left",0,10],
            ("가운데","가운데"): ["level","hover",0,10],
            ("가운데","오른쪽"): ["level","right",0,10],
            ("아래","왼쪽"): ["level","backward_left",0,10],
            ("아래","가운데"): ["level","backward",0,10],
            ("아래","오른쪽"): ["level","backward_right",0,10]
        }
        return mapping.get((vertical,horizontal),None)

     # 화재 지점 GPS 추정 함수
    def fire_gps(self, drone_gps, center_x, center_y):               
      # 화재 지점의 각도 계산 (가정: 카메라 시야각 90도, 프레임 중심에서 픽셀당 각도 계산)
      frame_center_x, frame_center_y = self.frame_width // 2, self.frame_height // 2
      pixels_per_degree = self.frame_width / 90.0
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
      if len(results[0].boxes) > 0:
        best_box = max(results[0].boxes, key=lambda box: box.conf[0])
        class_name = self.class_names[int(best_box.cls[0])]   
        if class_name.lower() in ["fire", "smoke"]:
          x1,y1,x2,y2 = best_box.xyxy[0]
          center_x = int((x1+x2)/2)
          center_y = int((y1+y2)/2)
          lat, lon = self.fire_gps(drone_gps, center_x, center_y)
          
            
      
