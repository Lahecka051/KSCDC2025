#nonfire_patrol.py
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
    def __init__(self, drone_system, landing, communicator):  # 수정: fire_detector 매개변수 제거
        self.drone_system = drone_system
        # self.fire_detector = fire_detector  # 수정: 제거
        self.landing = landing
        self.communicator = communicator
        self.status_q = queue.Queue()
        self.stop_event = threading.Event()
        self.ARRIVAL_RADIUS = 2
        self.drone_gps = DroneGPS(self.drone_system)

    def run(self, path):
        if not path: 
            print("[순찰] 경로가 없습니다.")
            return
            
        print("[순찰] GPS 이동 테스트 시작")
        
        # 시동 및 이륙
        print("[순찰] 시동 걸기...")
        if not self.drone_system.arm():
            print("[순찰] 시동 실패")
            return
        time.sleep(5)
        
        print("[순찰] 이륙 시작...")
        if not self.drone_system.takeoff():
            print("[순찰] 이륙 실패")
            self.drone_system.disarm()
            return
        time.sleep(5)

        # GPS 경로점 순회
        for i, waypoint in enumerate(path):
            lat = waypoint['lat']
            lon = waypoint['lon']
            print(f"\n[순찰] Waypoint {i+1} 이동 시작")
            print(f"  목표: 위도 {lat:.7f}, 경도 {lon:.7f}")
            
            while True:
                # GPS 업데이트
                self.drone_gps.update()
                
                # GPS 이동 명령
                self.drone_system.goto_gps(lat, lon)
                
                # 현재 위치 읽기
                current_coord = self.drone_system.read_gps()
                if current_coord:
                    current_lat = current_coord["lat"]
                    current_lon = current_coord["lon"]
                    
                    # 거리 계산
                    distance = self.drone_system.get_distance_metres(
                        current_lat, current_lon, lat, lon
                    )
                    
                    # 2초마다 상태 출력
                    if int(time.time()) % 2 == 0:
                        print(f"  현재 거리: {distance:.1f}m")
                    
                    # 목표 지점 도착
                    if distance <= self.ARRIVAL_RADIUS:
                        print(f"[순찰] Waypoint {i+1} 도착!")
                        time.sleep(2)
                        break
                
                time.sleep(0.1)
        
        # 홈으로 복귀
        print("\n[순찰] 모든 경로점 완료. 홈으로 복귀...")
        self.drone_system.goto_home()
        
        # 착륙
        print("[순찰] 착륙 시작...")
        self.landing.run()
        
        print("[순찰] GPS 이동 테스트 완료!")