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
        self.ball_count = 2
        self.servo = servo
        
    def drop_ball(self):
        """소화볼 투하 시퀀스"""
        self.ball_count -= 1
        print(f"[소화] 소화볼 투하 시작 (남은 개수: {self.ball_count})")
        
        # 수정: 투하 순서 간소화
        self.servo.drop()  # 0도 - 투하
        time.sleep(3)
        
        self.servo.idle()  # 90도 - idle 복귀
        time.sleep(1)
        
        print("[소화] 소화볼 투하 완료")
        
    def run(self, target):
        self.ball_count = 2
        self.servo.idle()  # 수정: idle() 메서드 사용 (90도)
        
        # GPS 이동 (target은 [latitude, longitude] 형식)
        if target and len(target) >= 2:
            print(f"화재 지점으로 이동: 위도 {target[0]}, 경도 {target[1]}")
            self.drone_system.goto_gps(target[0], target[1])
            
            # 도착 확인 로직
            while True:
                current_gps = self.drone_system.read_gps()
                if current_gps:
                    distance = self.drone_system.get_distance_metres(
                        current_gps['lat'], current_gps['lon'],
                        target[0], target[1]
                    )
                    if distance <= 2:  # 2미터 이내 도착
                        print("화재 지점 도착")
                        break
                time.sleep(0.5)
        
        # 객체 정렬
        align = False
        while not align:
            align, cmd = self.fire_detector.align_drone_to_object()
            
            # DroneController의 set_command 사용
            self.drone_system.set_command(cmd)
            
            if self.fire_detector.patrol_state == 'stop':
                break
                
        if align:
            print("화재 위치 정렬 완료. 소화탄 투하")
            self.drop_ball()
            
        # DroneController의 goto_home 사용
        print("스테이션으로 복귀")
        self.drone_system.goto_home()
        
        # 착륙
        self.landing.run()
        
        # 수정: 착륙 후 서보 초기 위치로
        self.servo.drop()  # 0도 - 투하 위치 (착륙 후 안전)
        
        self.communicator.send_status_update("EXTINGUISH_COMPLETE", self.ball_count)