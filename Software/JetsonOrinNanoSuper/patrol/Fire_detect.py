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

    def detect_fire_upper(self):
      ret, frame = self.cap0.read()
      if not ret: return False, None, None
      results = self.model.predict(frame, imgsz=920, conf=0.4, verbose=False)
      if len(results[0].boxes) > 0:
        best_box = max(results[0].boxes, key=lambda box: box.conf[0])
        class_name = self.class_names[int(best_box.cls[0])]
        
        if len(results[0].boxes) > 0:
        best_box = max(results[0].boxes, key=lambda box: box.conf[0])
        class_name = self.class_names[int(best_box.cls[0])]
        
        if class_name.lower() in ["fire", "smoke"]:
            
      
