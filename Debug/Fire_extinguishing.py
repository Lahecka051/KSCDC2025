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
        self.ball_count -= 1
        self.servo.set_angle(180)
        time.sleep(3)
        self.servo.set_angle(0)
        time.sleep(3)
        self.servo.set_angle(90)
        time.sleep(1)
        
    def run(self, target):
        self.ball_count = 2
        self.servo.set_angle(90)
        
        # GPS 이동 (target은 [latitude, longitude] 형식)
        if target and len(target) >= 2:
            print(f"화재 지점으로 이동: 위도 {target[0]}, 경도 {target[1]}")
            self.drone_system.goto_gps(target[0], target[1])  # 수정: success 반환값 제거 (goto_gps는 반환값이 없음)
            
            # 수정: 도착 확인 로직 추가
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
        self.servo.set_angle(0)
        self.communicator.send_status_update("EXTINGUISH_COMPLETE", self.ball_count)