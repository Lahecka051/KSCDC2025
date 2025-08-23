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

    def run(self, command_data):
    #여기에 gps좌표로 이동하는 함수
        fire_confirmed = False
        align = False
        while not fire_confirmed
            while not self.fire_detector.observing:
                Current_coordinates = #여기에 현재gps좌표를 읽어오는 함수
                fire_confirmed,_ = detect_fire_upper(Current_coordinates)
            # 드론을 호버링 시키는 함수
            time.sleep(5)
            fire_confirmed, fire_coords = detect_fire_upper(Current_coordinates)

        #여기에 fire_coords 좌표로 이동하는 함수 이동완료할때까지 다음 while문으로 안넘어가게 할 방법 필요
        while not align:
            align, cmd = align_drone_to_object()
            #cmd기준으로 드론을 움직이는 함수

        #여기에 현재gps좌표를 읽어오는 함수
        self.fire_detctor.capture_and_save_image()
        #여기에 드론 스테이션으로 돌아가게 하는 함수
        #착륙함수
        # gps좌표와 사진을 전송하는 함수
