#patrol.py
import socket
import json
import time
import cv2
import queue
import threading
import math

class DroneGPS:
    def __init__(self, drone_system):
        self.drone = drone_system
        self.altitude = 5.0
        self.heading = 0
        self.latitude = 0
        self.longitude = 0
        
    def update(self):
        gps_data = self.drone.read_gps()
        if gps_data:
            self.latitude = gps_data['lat']
            self.longitude = gps_data['lon']
        alt = self.drone.read_altitude()
        if alt:
            self.altitude = alt

class Patrol():
    def __init__(self, drone_system, fire_detector, landing, communicator):
        self.drone_system = drone_system
        self.fire_detector = fire_detector
        self.landing = landing
        self.communicator = communicator
        self.status_q = queue.Queue()
        self.stop_event = threading.Event()
        self.ARRIVAL_RADIUS = 2
        self.drone_gps = DroneGPS(self.drone_system)

    def run(self, path):
        if not path: return
        fire_confirmed = False
        align = False
        self.stop_event.clear()
        status = "MOVING_TO_TARGET"
        
        self.drone_system.arm()
        time.sleep(5)
        self.drone_system.takeoff()
        time.sleep(5)
        
        threading.Thread(target=self.fire_detector.fire_detection_thread,
                        args=(self.drone_gps, self.status_q, self.stop_event),
                        daemon=True).start()

        for i, waypoint in enumerate(path):
            lat = waypoint['lat']
            lon = waypoint['lon']
            print(f"경로점 {i+1} ({lat},{lon}) 이동 시작")
            
            while not fire_confirmed:  # 수정: 콜론 추가
                # 수정: drone_gps 업데이트
                self.drone_gps.update()
                
                try:
                    result = self.status_q.get_nowait()
                    if result["status"] == "started":
                        status = "OBSERVING"  # 수정: 오타 수정
                        
                    elif result["status"] == "recognized":
                        status = "FIRE_DETECTED"
                        fire_coords = result["coords"]
                        print("화제추정 지점:", fire_coords)
                        
                    elif result["status"] == "failed":
                        status = "MOVING_TO_TARGET"

                except queue.Empty:
                    pass

                if status == "OBSERVING":  # 수정: 오타 수정
                    print("호버링, 화재 관찰중")
                elif status == "FIRE_DETECTED":
                    fire_confirmed = True
                    self.stop_event.set()
                    print("화재 감지, 화재 지점으로 이동합니다")
                    break
                elif status == "MOVING_TO_TARGET":
                    self.drone_system.goto_gps(lat, lon)  # 수정: self.drone_system 사용
                    current_coord = self.drone_system.read_gps()
                    if current_coord:  # 수정: None 체크 추가
                        current_lat = current_coord["lat"]
                        current_lon = current_coord["lon"]

                        distance = self.drone_system.get_distance_metres(current_lat, current_lon, lat, lon)
                
                        # 목표 지점에 도달하면 break
                        if distance <= self.ARRIVAL_RADIUS:
                            print(f"waypoint{i+1} 도착")
                            break
            if fire_confirmed:
                break

        if fire_confirmed:
            fire_lat = fire_coords[0]
            fire_lon = fire_coords[1]
            while True:
                self.drone_system.goto_gps(fire_lat, fire_lon)  # 수정: self.drone_system 사용
                current_coord = self.drone_system.read_gps()  # 수정: 현재 좌표 다시 읽기
                if current_coord:
                    current_lat = current_coord["lat"]
                    current_lon = current_coord["lon"]
                    distance_fire = self.drone_system.get_distance_metres(current_lat, current_lon, fire_lat, fire_lon)  # 수정: fire_lat, fire_lon 사용
                    
                    # 목표 지점에 도달하면 break
                    if distance_fire <= self.ARRIVAL_RADIUS:
                        print(f"화재 지점 근처 도착")
                        break
                
            while not align:
                align, cmd = self.fire_detector.align_drone_to_object()  # 수정: self.fire_detector 사용
                self.drone_system.set_command(cmd)  # 수정: cmd 매개변수 추가
                if self.fire_detector.patrol_state == 'stop':
                    break
                    
            if not self.fire_detector.patrol_state == 'stop':
                coordinates = self.drone_system.read_gps()
                self.fire_detector.capture_and_save_image()  # 수정: self.fire_detector 사용
                
        self.drone_system.goto_home()
        self.landing.run()
        
        if fire_confirmed:
            image_path = "captured_image.jpg"  # 수정: image_path 정의
            self.communicator.send_fire_report(coordinates, image_path)  # 수정: 콜론 제거



