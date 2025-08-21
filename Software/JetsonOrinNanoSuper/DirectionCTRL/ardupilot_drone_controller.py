"""
ardupilot_drone_controller.py
ArduPilot (FC H743v2) 전용 드론 제어 시스템
쓰로틀 값 조정: 90%(상승), 70%(호버링), 60%(하강)
"""

from pymavlink import mavutil
import time
import threading
import math
from typing import Optional, Tuple

class ArduPilotDroneController:
    """ArduPilot 전용 4가지 데이터 제어"""
    
    def __init__(self, connection_string='/dev/ttyTHS1', baudrate=115200):
        """
        초기화
        
        Args:
            connection_string: UART 포트 (젯슨 -> FC)
            baudrate: 통신 속도 (ArduPilot 기본 115200)
        """
        self.master = None
        self.connection_string = connection_string
        self.baudrate = baudrate
        
        # 현재 명령
        self.current_command = ["level", "hover", 0, 70]  # 기본 호버링 70%
        self.is_armed = False
        self.is_hovering = False  # 호버링 상태 플래그
        self.control_thread = None
        self.running = False
        
        # 쓰로틀 3단계 설정 (ArduPilot PWM)
        # 90% = 1700 PWM (상승/이륙)
        # 70% = 1600 PWM (호버링/이동)
        # 60% = 1500 PWM (하강/착륙)
        self.THROTTLE_MAP = {
            60: 1500,   # 하강/착륙
            70: 1600,   # 호버링/이동
            90: 1700    # 상승/이륙
        }
        
        # GPS 관련
        self.gps_thread = None
        self.gps_active = False
        
    def connect(self) -> bool:
        """FC 연결"""
        try:
            print(f"ArduPilot FC 연결 중... {self.connection_string}")
            
            # MAVLink 연결
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            
            # Heartbeat 대기
            print("Heartbeat 대기 중...")
            self.master.wait_heartbeat()
            print(f"FC 연결 성공! (시스템 ID: {self.master.target_system})")
            
            # ArduPilot 파라미터 설정 (실내용)
            self._setup_indoor_params()
            
            # 제어 스레드 시작
            self.running = True
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            
            return True
            
        except Exception as e:
            print(f"연결 실패: {e}")
            return False
    
    def _setup_indoor_params(self):
        """실내 비행용 ArduPilot 파라미터 설정"""
        print("실내 비행 파라미터 설정...")
        
        params = {
            'ARMING_CHECK': 0,      # 모든 체크 비활성화
            'GPS_TYPE': 0,          # GPS 비활성화
            'AHRS_GPS_USE': 0,      # AHRS GPS 사용 안함
            'EK3_GPS_TYPE': 0,      # EKF3 GPS 비활성화
            'FENCE_ENABLE': 0,      # 지오펜스 비활성화
            'FS_GCS_ENABLE': 0,     # GCS 페일세이프 비활성화
            'FS_THR_ENABLE': 0,     # 쓰로틀 페일세이프 비활성화
        }
        
        for param, value in params.items():
            try:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    param.encode('utf-8'),
                    value,
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                time.sleep(0.1)
            except:
                pass
        
        print("파라미터 설정 완료")
    
    def arm(self) -> bool:
        """시동 걸기"""
        print("시동 걸기...")
        
        # STABILIZE 모드 설정 (GPS 불필요)
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0  # STABILIZE = 0
        )
        time.sleep(1)
        
        # ARM 명령
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1=arm, 0=disarm
            0, 0, 0, 0, 0, 0
        )
        
        # 응답 대기
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.is_armed = True
            print("시동 성공!")
            
            # 시동 후 5초 대기 (안정화)
            print("시동 안정화 대기 중... (5초)")
            time.sleep(5)
            
            # 시동 후 기본 호버링 상태로 설정 (70%)
            self.is_hovering = True
            self.current_command = ["level", "hover", 0, 70]
            print("준비 완료!")
            
            return True
        else:
            print("시동 실패")
            return False
    
    def disarm(self) -> bool:
        """시동 끄기 (개선된 버전)"""
        print("시동 끄기 시작...")
        
        # 1. 모든 이동 중지
        self.is_hovering = False
        self.gps_active = False
        self.running = False  # 제어 루프 중지
        
        # 2. 쓰로틀을 완전히 최소값으로 (여러 번 전송)
        print("쓰로틀 0으로 설정 중...")
        for _ in range(5):
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                1500,   # CH1 - Roll (중립)
                1500,   # CH2 - Pitch (중립)  
                1000,   # CH3 - Throttle (최소)
                1500,   # CH4 - Yaw (중립)
                0, 0    # CH5-6 (사용 안함)
            )
            time.sleep(0.2)
        
        # 3. LAND 모드로 전환 (착륙 모드)
        print("LAND 모드 전환...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9  # LAND = 9
        )
        time.sleep(2)
        
        # 4. 강제 DISARM 명령 (21196 = 강제 disarm)
        print("강제 시동 끄기 명령 전송...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,      # 0=disarm
            21196,  # 강제 disarm 매직 넘버
            0, 0, 0, 0, 0
        )
        
        # 응답 대기
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        
        # 5. 일반 DISARM 재시도
        if not msg or msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("일반 시동 끄기 재시도...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # 0=disarm
                0, 0, 0, 0, 0, 0
            )
            time.sleep(1)
        
        # 6. RC Override 해제
        print("RC Override 해제...")
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0  # 모든 채널 해제
        )
        
        self.is_armed = False
        print("시동 끄기 완료!")
        return True
    
    def set_command(self, vertical: str, horizontal: str, rotation: int, throttle_percent: int):
        """
        4가지 데이터로 드론 제어
        
        Args:
            vertical: "up", "level", "down"
            horizontal: "forward", "backward", "left", "right", 
                       "forward_left", "forward_right", 
                       "backward_left", "backward_right", "hover"
            rotation: 0-359 (시계방향 회전)
            throttle_percent: 60(하강), 70(호버링/이동), 90(상승)만 사용
        """
        if not self.is_armed:
            print("시동이 걸려있지 않음")
            return False
        
        # 쓰로틀 값 검증 (60, 70, 90만 허용)
        if throttle_percent not in [60, 70, 90]:
            print(f"잘못된 쓰로틀 값: {throttle_percent}. 60, 70, 90만 사용 가능")
            return False
        
        # 호버링 상태 해제 (새 명령이 왔으므로)
        self.is_hovering = False
        
        # GPS 이동 취소
        if self.gps_active:
            self.gps_active = False
            print("GPS 이동 취소됨")
        
        self.current_command = [vertical, horizontal, rotation, throttle_percent]
        
        # 상태 설명
        throttle_desc = {
            60: "하강/착륙",
            70: "호버링/이동",
            90: "상승/이륙"
        }
        
        print(f"명령: [{vertical}, {horizontal}, {rotation}°, {throttle_percent}%({throttle_desc[throttle_percent]})]")
        return True
    
    def _control_loop(self):
        """제어 루프 (백그라운드)"""
        while self.running:
            try:
                if self.is_armed:
                    # 호버링 상태면 강제로 호버링 명령 유지 (70%)
                    if self.is_hovering:
                        vertical, horizontal, rotation, throttle_percent = "level", "hover", 0, 70
                    else:
                        vertical, horizontal, rotation, throttle_percent = self.current_command
                    
                    # PWM 값 계산
                    throttle_pwm = self.THROTTLE_MAP.get(throttle_percent, 1600)  # 기본값 70%
                    roll_pwm = 1500      # 중립
                    pitch_pwm = 1500     # 중립
                    yaw_pwm = 1500       # 중립
                    
                    # Horizontal (이동) - 호버링이 아닐 때만
                    if not self.is_hovering and horizontal != "hover":
                        move_amount = 200  # 이동 시 고정값
                        
                        if horizontal == "forward":
                            pitch_pwm = 1500 - move_amount
                        elif horizontal == "backward":
                            pitch_pwm = 1500 + move_amount
                        elif horizontal == "left":
                            roll_pwm = 1500 - move_amount
                        elif horizontal == "right":
                            roll_pwm = 1500 + move_amount
                        elif horizontal == "forward_left":
                            pitch_pwm = 1500 - int(move_amount * 0.7)
                            roll_pwm = 1500 - int(move_amount * 0.7)
                        elif horizontal == "forward_right":
                            pitch_pwm = 1500 - int(move_amount * 0.7)
                            roll_pwm = 1500 + int(move_amount * 0.7)
                        elif horizontal == "backward_left":
                            pitch_pwm = 1500 + int(move_amount * 0.7)
                            roll_pwm = 1500 - int(move_amount * 0.7)
                        elif horizontal == "backward_right":
                            pitch_pwm = 1500 + int(move_amount * 0.7)
                            roll_pwm = 1500 + int(move_amount * 0.7)
                        
                        # Rotation (Yaw)
                        if rotation > 0:
                            yaw_rate = int((rotation / 360.0) * 500)
                            yaw_pwm = 1500 + yaw_rate
                    
                    # RC Override 전송 (ArduPilot 방식)
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system,
                        self.master.target_component,
                        roll_pwm,     # CH1
                        pitch_pwm,    # CH2
                        throttle_pwm, # CH3
                        yaw_pwm,      # CH4
                        0, 0, 0, 0    # CH5-8 (사용 안함)
                    )
                
                time.sleep(0.05)  # 20Hz
                
            except Exception as e:
                print(f"제어 오류: {e}")
                time.sleep(0.1)
    
    def brake(self):
        """긴급 정지 / 호버링"""
        print("\n!!! 브레이크 !!! - 모든 이동 중지, 호버링 시작")
        
        # 모든 이동 명령 취소
        self.gps_active = False  # GPS 이동 취소
        
        # 호버링 상태 활성화
        self.is_hovering = True
        
        # 즉시 중립 위치로 설정 (70% 쓰로틀 유지)
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500,                         # CH1 - Roll (중립)
            1500,                         # CH2 - Pitch (중립)
            self.THROTTLE_MAP[70],        # CH3 - Throttle (70%)
            1500,                         # CH4 - Yaw (중립)
            0, 0, 0, 0
        )
        
        print(f"호버링 모드 활성화 (쓰로틀 70% = {self.THROTTLE_MAP[70]} PWM)")
        print("다음 명령 대기 중...")
    
    def close(self):
        """연결 종료"""
        self.running = False
        self.is_hovering = False
        self.gps_active = False
        
        if self.control_thread:
            self.control_thread.join(timeout=1)
        if self.gps_thread:
            self.gps_thread.join(timeout=1)
        if self.master:
            self.master.close()


# ============= 사용 예제 =============

def main():
    # 드론 컨트롤러 생성
    drone = ArduPilotDroneController('/dev/ttyTHS1', 115200)
    
    if not drone.connect():
        return
    
    try:
        # 시동 (5초 대기 포함)
        print("\n=== 드론 테스트 시작 ===")
        drone.arm()
        
        # 상승 (90% 쓰로틀, 5초)
        print("\n[1] 상승 시작 (90% 쓰로틀, 5초)...")
        drone.set_command("up", "hover", 0, 90)
        time.sleep(5)
        
        # 2초 딜레이
        print("\n2초 대기...")
        time.sleep(2)
        
        # 호버링 (70% 쓰로틀, 5초)
        print("\n[2] 호버링 (70% 쓰로틀, 5초)...")
        drone.set_command("level", "hover", 0, 70)
        time.sleep(5)
        
        # 2초 딜레이
        print("\n2초 대기...")
        time.sleep(2)
        
        # 하강 (60% 쓰로틀, 5초)
        print("\n[3] 하강 (60% 쓰로틀, 5초)...")
        drone.set_command("down", "hover", 0, 60)
        time.sleep(5)
        
        # 2초 딜레이
        print("\n2초 대기...")
        time.sleep(2)
        
        # 시동 끄기
        print("\n[4] 시동 끄기...")
        drone.disarm()
        
        print("\n=== 테스트 완료 ===")
        
    except KeyboardInterrupt:
        print("\n\n긴급 중단!")
        drone.brake()
        time.sleep(1)
        drone.disarm()
    
    finally:
        drone.close()
        print("프로그램 종료")

if __name__ == "__main__":
    main()
