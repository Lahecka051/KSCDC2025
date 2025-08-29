#patrol.py
import socket
import json
import time
import cv2
import queue
import threading
import math

class Patrol():
    def __init__(self, drone_system, fire_detector, landing, communicator):
        self.drone_system = drone_system
        self.fire_detector = fire_detector
        self.landing = landing
        self.communicator = communicator
        self.status_q = queue.Queue()
        self.stop_event = threading.Event()

    def run(self, path):
        if not path: return
        fire_confirmed = False
        align = False
        self.stop_event.clear()
        status = "MOVING_TO_TARGET"
        threading.Thread(target=self.fire-detector.fire_detection_thread,args=(drone_gps, self.status_q,self.stop_event),daemon=True).start()
        self.drone_system.arm()
        self.drone_system.takeoff()

        for i, waypoint in enumerate(path):
            lat = waypoint['lat']
            lon = waypoint['lon']
            print(f"경로점 {i+1} ({lat},{lon}) 이동 시작")
            while not fire_confirmed
                try:
                    result = self.status_q.get_nowait()
                    if result["status"] == "started":
                        status = "OBSERVIG"
                        
                    elif result["status"] == "recognized":
                        status = "FIRE_DETECTED"
                        fire_coords = result["coords"]
                        
                    elif result["status"] == "failed":
                        status = "MOVING_TO_TARGET"

                except queue.Empty:
                    pass

                if status == "OBSERVIG":
                    print("호버링, 화재 관찰중")
                elif status == "FIRE_DETECTED":
                    fire_confirmed = True
                    self.stop_event.set()
                    print("화재 감지, 화재 지점으로 이동합니다")
                    break
                elif status == "MOVING_TO_TARGET":
                    goto_gps(lat,lon)
                    current_coord = self.drone_system.read_gps()
                    current_lat = current_coord["lat"]
                    current_lon = current_coord["lon"]

                    distance = self.drone_system.get_distance_metres(current_lat,current_lon,lat,lon)
                
                #목표 지점에 도달하면 break
                if distance <= self.ARRIVAL_RADIUS:
                    print(f"waypoint{i+1} 도착")
                    break

        if fire_confirmed:
            fire_lat = fire_coords[0]
            fire_lon = fire_coords[1]
            while True:
                goto_gps(fire_lat,fire_lon)
                distance_fire = self.drone_system.get_distance_metres(current_lat,current_lon,lat,lon)
                
                #목표 지점에 도달하면 break
                if distance_fire <= self.ARRIVAL_RADIUS:
                    print(f"화제 지점 근처 도착")
                    break
                
            while not align:
                align, cmd = align_drone_to_object()
                self.drone_system.set_command()
                if self.fire_detector.patrol_state == 'stop':
                    break
            if not self.fire_detector.patrol_state == 'stop':
                coordinates = self.drone_system.read_gps()
                self.fire_detctor.capture_and_save_image()
        self.drone_system.goto_home()
        self.landing.run()
        if fire_confirmed:
            self.communicator.send_fire_report(coordinates, image_path):
