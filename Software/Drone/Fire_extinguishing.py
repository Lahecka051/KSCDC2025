import socket
import json
import time
import cv2

# [최적화] 도착 판정 상수 정의
ARRIVAL_RADIUS = 2          # 도착 판정 반경 (미터)
ARRIVAL_TIMEOUT = 60        # 도착 대기 최대 시간 (초)

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

        self.servo.drop()   # 180도 - 투하
        time.sleep(3)

        self.servo.reload() # 0도 - idle 복귀
        time.sleep(3)

    def run(self, target):
        self.servo.reload()  # 0도 - idle 복귀

        if target and 'lat' in target and 'lon' in target:
            lat = target['lat']
            lon = target['lon']
            print(f"화재 지점으로 이동: 위도 {lat}, 경도 {lon}")

            self.drone_system.arm()
            time.sleep(5)
            self.drone_system.takeoff()
            time.sleep(5)

            # [최적화] 도착 확인 루프에 타임아웃 추가 + goto_gps 루프 내부 이동
            # 기존: goto_gps가 while 밖에서 1회만 호출 → FC가 명령을 놓치면 이동하지 않음
            #       또한 while True 무한 루프 — GPS 장애 시 탈출 불가
            # 수정: 매 반복마다 goto_gps 호출하여 FC에 지속적으로 목표 전송 + 타임아웃 추가
            arrival_start = time.time()
            while time.time() - arrival_start < ARRIVAL_TIMEOUT:
                self.drone_system.goto_gps(lat, lon)
                current_gps = self.drone_system.read_gps()
                if current_gps:
                    distance = self.drone_system.get_distance_metres(
                        current_gps['lat'], current_gps['lon'],
                        lat, lon
                    )
                    if distance <= ARRIVAL_RADIUS:
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
