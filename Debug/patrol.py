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
        self.altitude = 2.0
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
            # 수정: 한글 제거
            print(f"Waypoint {i+1} ({lat},{lon}) starting")
            
            while not fire_confirmed:
                # drone_gps 업데이트
                self.drone_gps.update()
                
                try:
                    result = self.status_q.get_nowait()
                    if result["status"] == "started":
                        status = "OBSERVING"
                        
                    elif result["status"] == "recognized":
                        status = "FIRE_DETECTED"
                        fire_coords = result["coords"]
                        self.fire_detector.capture_and_save_image()
                        # 수정: 한글 제거
                        print("Estimated fire location:", fire_coords)
                        
                    elif result["status"] == "failed":
                        status = "MOVING_TO_TARGET"

                except queue.Empty:
                    pass

                if status == "OBSERVING":
                    # 수정: 한글 제거
                    print("Hovering, observing fire")
                elif status == "FIRE_DETECTED":
                    fire_confirmed = True
                    self.stop_event.set()
                    # 수정: 한글 제거
                    print("Fire detected, moving to fire location")
                    break
                elif status == "MOVING_TO_TARGET":
                    self.drone_system.goto_gps(lat, lon)
                    current_coord = self.drone_system.read_gps()
                    if current_coord:
                        current_lat = current_coord["lat"]
                        current_lon = current_coord["lon"]

                        distance = self.drone_system.get_distance_metres(current_lat, current_lon, lat, lon)
                
                        # 목표 지점에 도달하면 break
                        if distance <= self.ARRIVAL_RADIUS:
                            # 수정: 한글 제거
                            print(f"Waypoint {i+1} arrived")
                            break
            if fire_confirmed:
                break
                
        self.drone_system.goto_home()
        self.landing.run()
        
        if fire_confirmed:
            image_path = "captured_image.jpg"

            self.communicator.send_fire_report(coordinates, image_path)
