#patrol.py
import socket
import json
import time
import cv2

class Patrol():
  def __init__(self, drone_system: IntegratedDroneSystem, fire_detector, landing, communicator):
    self.drone_system = drone_system
    self.fire_detector = fire_detector
    self.landing = landing
    self.communicator = communicator
