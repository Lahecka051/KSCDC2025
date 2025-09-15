import socket
import json
import time
import cv2

class Fire_extinguishing():
    def __init__(self, drone_system, fire_detector, landing, communicator, servo):
        self.drone_system = drone_system
        self.fire_detector = fire_detector
        self.landing = landing
        self.communicator = communicator
        self.servo = servo
        
    def drop_ball(self):
        """소화볼 투하 시퀀스"""
        print("[소화] 소화볼 투하 시작")
        
        self.servo.drop()  # 180도 - 투하
        time.sleep(3)
        
        self.servo.reload()  # 0도 - idle 복귀
        time.sleep(3)
        
    def run(self, target):
        self.servo.reload()  # 0도 - idle 복귀

        # #수정: 딕셔너리 타입으로 올바르게 접근
        if target and 'lat' in target and 'lon' in target:
            lat = target['lat']
            lon = target['lon']
            print(f"화재 지점으로 이동: 위도 {lat}, 경도 {lon}")
            
            # #수정: arm과 takeoff 추가
            self.drone_system.arm()
            time.sleep(5)
            self.drone_system.takeoff()
            time.sleep(5)
            
            self.drone_system.goto_gps(lat, lon)
            
            # 도착 확인 로직
            while True:
                current_gps = self.drone_system.read_gps()
                if current_gps:
                    distance = self.drone_system.get_distance_metres(
                        current_gps['lat'], current_gps['lon'],
                        lat, lon
                    )
                    if distance <= 2:  # 2미터 이내 도착
                        print("화재 지점 도착")
                        break
                time.sleep(0.5)
        
        # 객체 정렬
        align = False
        while not align:
            align, cmd = self.fire_detector.align_drone_to_object()
            
            self.drone_system.set_command(cmd)
            
            if self.fire_detector.patrol_state == 'stop':
                break
                
        if align:
            print("화재 위치 정렬 완료. 소화탄 투하")
            self.drop_ball()
            
        print("스테이션으로 복귀")
        self.drone_system.goto_home()
        
        # 착륙
        self.landing.run()
        
        self.servo.reload()  # 0도 - 재장전 위치
        
        self.communicator.send_status_update("EXTINGUISH_COMPLETE", None)