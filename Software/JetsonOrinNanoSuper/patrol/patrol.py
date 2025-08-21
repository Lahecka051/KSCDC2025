#patrol.py
import socket
import json
import time
import cv2

class patrol():
  def __init__(self, drone_system: IntegratedDroneSystem, cap0, cap1, object_detector, landing, communicator):
    self.drone_system = drone_system
    self.cap0 = cap0
    self.cap1 = cap1
    self.object_detector = object_detector
    self.landing = landing
    self.communicator = communicator
