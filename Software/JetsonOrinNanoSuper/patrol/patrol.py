#patrol.py
import socket
import json
import time
import cv2

class Patrol():
  def __init__(self, drone_system: IntegratedDroneSystem, object_detector, landing, communicator):
    self.drone_system = drone_system
    self.object_detector = object_detector
    self.landing = landing
    self.communicator = communicator
