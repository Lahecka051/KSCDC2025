#!/usr/bin/env python3
"""
standalone_flight_demo.py
단독 실행 비행 데모 - 이륙, 호버링, 전진/후진, 착륙
3가지 고도 정보 실시간 출력
"""

from pymavlink import mavutil
import time
import sys
import threading

class SimpleDroneController:
    """간단한 드론 컨트롤러 - 다중 고도 센서 모니터링"""
    
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=115200):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.is_armed = False
        
        # 3가지 고도 정보
        self.gps_altitude = 0.0      # GPS 해발고도 (절대고도)
        self.baro_altitude = 0.0     # 기압계 고도 (VFR_HUD)
        self.ekf_altitude = 0.0      # EKF 융합 고도 (LOCAL_POSITION_NED)
        self.relative_alt = 0.0      # 상대 고도 (이륙 지점 기준)
        
        # 모니터링 스레드
        self.monitoring = False
        self.monitor_thread = None
        
    def connect(self):
        """FC 연결"""
        try:
            print("[드론] FC 연결 중...")
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            self.master.wait_heartbeat()
            print("[드론] ✅ FC 연결 성공")
            
            # 데이터 스트림 요청
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                50,  # 50Hz로 데이터 요청
                1
            )
            
            # 고도 모니터링 시작
            self.start_altitude_monitoring()
            return True
            
        except Exception as e:
            print(f"[드론] ❌ 연결 실패: {e}")
            return False
    
    def start_altitude_monitoring(self):
        """고도 실시간 모니터링 시작"""
        self.monitoring = True
        
        def monitor():
            while self.monitoring:
                try:
                    # VFR_HUD - 기압계 고도
                    msg = self.master.recv_match(type='VFR_HUD', blocking=False)
                    if msg:
                        self.baro_altitude = msg.alt
                    
                    # GLOBAL_POSITION_INT - GPS 고도
                    msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if msg:
                        self.gps_altitude = msg.alt / 1000.0  # mm를 m로 변환
                        self.relative_alt = msg.relative_alt / 1000.0  # 상대 고도
                    
                    # LOCAL_POSITION_NED - EKF 융합 고도
                    msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                    if msg:
                        self.ekf_altitude = -msg.z  # NED에서 z는 아래가 양수이므로 반전
                    
                    # HEARTBEAT - 시동 상태
                    msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
                    if msg and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1: # TEL1 255 190
                        self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    
                    time.sleep(0.02)  # 50Hz
                except:
                    pass
        
        self.monitor_thread = threading.Thread(target=monitor, daemon=True)
        self.monitor_thread.start()
    
    def print_altitudes(self):
        """3가지 고도 정보 출력"""
        print(f"  [고도] GPS해발: {self.gps_altitude:7.2f}m | "
              f"기압계: {self.baro_altitude:7.2f}m | "
              f"EKF융합: {self.ekf_altitude:7.2f}m | "
              f"상대: {self.relative_alt:6.2f}m", end='\r')
    
    def set_mode_guided(self):
        """GUIDED 모드 설정"""
        print("[드론] GUIDED 모드 설정...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode
        )
        time.sleep(1)
    
    def arm(self):
        """시동"""
        print("[드론] 시동 걸기...")
        
        # GUIDED 모드 설정
        self.set_mode_guided()
        
        # ARM 명령
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        # 시동 확인
        for i in range(10):
            if self.is_armed:
                print("[드론] ✅ 시동 성공")
                print("\n[초기 고도 상태]")
                self.print_altitudes()
                print()
                return True
            time.sleep(1)
        
        print("[드론] ❌ 시동 실패")
        return False
    
    def takeoff(self, altitude):
        """이륙 - GUIDED 모드는 이륙 후 자동 호버링"""
        print(f"\n[드론] 이륙 명령 (목표: {altitude}m)...")
        
        # TAKEOFF 명령
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        
        # 이륙 중 고도 모니터링 (8초간)
        print("[이륙 중 - 고도 모니터링]")
        for i in range(8):
            self.print_altitudes()
            print(f" | 경과시간: {i+1}초", end='\r')
            time.sleep(1)
        
        print(f"\n[드론] ✅ 이륙 완료 (8초 경과) - GUIDED 모드 자동 호버링")
    
    def move_forward(self, distance, speed=1.0):
        """전진 이동"""
        print(f"\n[드론] 전진 {distance}m (속도: {speed}m/s)...")
        
        # 전진 속도 명령 (X축 양의 방향)
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # 속도만 사용
            0, 0, 0,  # 위치
            speed, 0, 0,  # vx=전진, vy=0, vz=0
            0, 0, 0,  # 가속도
            0, 0      # yaw, yaw_rate
        )
        
        # 이동 시간 계산 (거리 / 속도)
        move_time = distance / speed
        
        print(f"[전진 중 - {move_time:.1f}초간 이동]")
        for i in range(int(move_time)):
            self.print_altitudes()
            print(f" | 전진중: {i+1}/{int(move_time)}초", end='\r')
            time.sleep(1)
        
        # 정지 (속도 0)
        self.stop_movement()
        print(f"\n[드론] ✅ 전진 {distance}m 완료")
    
    def move_backward(self, distance, speed=1.0):
        """후진 이동"""
        print(f"\n[드론] 후진 {distance}m (속도: {speed}m/s)...")
        
        # 후진 속도 명령 (X축 음의 방향)
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # 속도만 사용
            0, 0, 0,  # 위치
            -speed, 0, 0,  # vx=후진(음수), vy=0, vz=0
            0, 0, 0,  # 가속도
            0, 0      # yaw, yaw_rate
        )
        
        # 이동 시간 계산
        move_time = distance / speed
        
        print(f"[후진 중 - {move_time:.1f}초간 이동]")
        for i in range(int(move_time)):
            self.print_altitudes()
            print(f" | 후진중: {i+1}/{int(move_time)}초", end='\r')
            time.sleep(1)
        
        # 정지 (속도 0)
        self.stop_movement()
        print(f"\n[드론] ✅ 후진 {distance}m 완료")
    
    def stop_movement(self):
        """이동 정지 - 속도를 0으로 설정 (GUIDED 모드 자동 호버링)"""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # 속도만 사용
            0, 0, 0,  # 위치
            0, 0, 0,  # 속도 (모두 0 = 정지/호버링)
            0, 0, 0,  # 가속도
            0, 0      # yaw, yaw_rate
        )
    
    def land(self):
        """착륙"""
        print("\n[드론] 착륙 시작...")
        
        # LAND 모드 설정
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9  # LAND mode
        )
        
        # 착륙 중 고도 모니터링
        print("[착륙 중 - 고도 모니터링]")
        for i in range(15):
            self.print_altitudes()
            print(f" | 남은시간: {15-i:2d}초", end='\r')
            time.sleep(1)
        
        print("\n[드론] ✅ 착륙 완료 (15초 경과)")
    
    def disarm(self):
        """시동 끄기"""
        print("\n[드론] 시동 끄기...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        time.sleep(2)
        
        if not self.is_armed:
            print("[드론] ✅ 시동 해제 성공")
            return True
        
        print("[드론] ⚠️ 시동 해제 실패")
        return False
    
    def stop_monitoring(self):
        """모니터링 중지"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1)


def main():
    """메인 실행 함수"""
    
    print("="*70)
    print("   드론 비행 데모 - 다중 고도 센서 모니터링")
    print("   이륙(2m) → 호버링(15초) → 전진(1m) → 후진(1m) → 착륙")
    print("="*70)
    print("\n[고도 정보 설명]")
    print("  - GPS해발: GPS 기반 절대 고도 (해발 기준)")
    print("  - 기압계: 기압 센서 측정 고도")
    print("  - EKF융합: Extended Kalman Filter 융합 고도")
    print("  - 상대: 이륙 지점 기준 상대 고도")
    print("="*70)
    
    # 드론 객체 생성
    drone = SimpleDroneController('/dev/ttyACM0', 115200)
    
    try:
        # ========== 1. 연결 ==========
        print("\n[1단계] FC 연결")
        if not drone.connect():
            print("연결 실패. 프로그램 종료.")
            return
        time.sleep(2)  # 초기화 대기
        
        # 초기 고도 확인
        print("\n[연결 후 초기 고도]")
        for _ in range(3):
            drone.print_altitudes()
            print()
            time.sleep(1)
        
        # ========== 2. 시동 ==========
        print("\n[2단계] 시동")
        if not drone.arm():
            print("시동 실패. 프로그램 종료.")
            return
        time.sleep(2)
        
        # ========== 3. 이륙 (2미터) ==========
        print("\n[3단계] 이륙")
        drone.takeoff(2.0)
        time.sleep(3)  # 안정화 대기
        
        # ========== 4. 호버링 (5초) - GUIDED 모드 자동 호버링 ==========
        print("\n[4단계] 호버링 (5초) - GUIDED 모드 자동 호버링")
        print("[호버링 중 - 고도 모니터링]")
        print("※ GUIDED 모드에서는 별도 명령 없이도 자동으로 호버링됩니다")

        for i in range(5, 0, -1):
            drone.print_altitudes()
            print(f" | 남은시간: {i:2d}초", end='\r')
            time.sleep(1)  # GUIDED 모드가 자동으로 위치 유지
        print("\n✅ 호버링 완료")
        
        # ========== 5. 전진 1미터 ==========
        print("\n[5단계] 전진 이동")
        drone.move_forward(5.0, speed=1.0)  # 5m 전진, 속도 1.0m/s
        time.sleep(5)  # 안정화
        
        # ========== 6. 후진 1미터 ==========
        print("\n[6단계] 후진 이동")
        drone.move_backward(5.0, speed=1.0)  # 5m 후진, 속도 1.0m/s
        time.sleep(5)  # 안정화

        # ========== 7. 착륙 ==========
        print("\n[7단계] 착륙")
        drone.land()  # 착륙 명령 후 충분한 시간 대기
        time.sleep(3)  # 착륙 후 추가 안정화
        
        # ========== 8. 시동 해제 ==========
        print("\n[8단계] 시동 해제")
        drone.disarm()
        
        # 모니터링 중지
        drone.stop_monitoring()
        
        # ========== 완료 ==========
        print("\n" + "="*70)
        print("   ✅ 비행 데모 완료!")
        print("="*70)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자 중단!")
        print("긴급 착륙 시작...")
        
        # 긴급 착륙
        if drone.is_armed:
            # RTL 모드로 안전한 복귀
            drone.master.mav.set_mode_send(
                drone.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6  # RTL mode
            )
            print("RTL 모드 활성화 - 홈으로 복귀 중...")
            
            # 복귀 중 고도 모니터링
            for _ in range(20):
                drone.print_altitudes()
                print()
                time.sleep(1)
            
            print("안전하게 착륙했습니다.")
        
        drone.stop_monitoring()
        
    except Exception as e:
        print(f"\n\n❌ 오류 발생: {e}")
        print("긴급 착륙...")
        
        if drone.is_armed:
            # LAND 모드 설정
            drone.master.mav.set_mode_send(
                drone.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                9  # LAND mode
            )
            time.sleep(15)  # 충분한 착륙 시간
            drone.disarm()
        
        drone.stop_monitoring()


if __name__ == "__main__":
    print("\n드론 비행 데모 - 다중 고도 센서 모니터링")
    print("실행 시작...\n")
    main()
