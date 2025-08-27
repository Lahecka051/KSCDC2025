#Fire_extinguishing.py
import socket
import json
import time
import cv2
from MG90 import MG90

class Fire_extinguishing():
    def __init__(self, drone_system, fire_detector, landing, communicator, servo):
        self.drone_system = drone_system
        self.fire_detector = fire_detector
        self.landing = landing
        self.communicator = communicator
        self.ball_count = 2
        self.servo = servo

    def drop_ball(self):
        self.ball_count -=1
        self.servo.set_angle(180)
        time.sleep(3)
        self.servo.set_angle(0)
        time.sleep(3)
        self.servo.set_angle(90)
        time.sleep(1)

    def run(self,target):
        self.ball_count = 2
        self.servo.set_angle(90)
        #여기에 gps(target) 지점으로 이동하는 함수 이동 완료시 전까지 다음 while 문으로 안넘어가야함
        while not align:
            align, cmd = align_drone_to_object()
            #cmd기준으로 드론을 움직이는 함수
            if self.fire_detector.patrol_state == 'stop':
                break
        if align:
            self.drop_ball()
            
        #스테이션으로 복귀하는 함
        self.landing.run()
        self.servo.set_angle(0)
        self.communicator.send_status_update("EXTINGUISH_COMPLETE", ball_count)
