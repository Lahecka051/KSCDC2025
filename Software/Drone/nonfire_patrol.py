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
    def __init__(self, drone_system, landing, communicator):
        self.drone_system = drone_system
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
            
            # 수정: 상태 출력용 변수
            last_print_time = 0
            
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
                    
                    # 수정: 현재 고도 읽기
                    current_altitude = self.drone_gps.altitude
                    
                    # 수정: 0.5초마다 같은 줄에 상태 업데이트
                    current_time = time.time()
                    if current_time - last_print_time >= 0.5:
                        # \r로 커서를 줄 처음으로, \033[K로 줄 끝까지 지우기
                        print(f"\r\033[K  거리: {distance:.1f}m | 고도: {current_altitude:.1f}m", end='', flush=True)
                        last_print_time = current_time
                    
                    # 목표 지점 도착
                    if distance <= self.ARRIVAL_RADIUS:
                        print(f"\r\033[K[순찰] Waypoint {i+1} 도착!")  # 수정: 새 줄로 도착 메시지
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