"""
ardupilot_drone_controller.py
ArduPilot (FC H743v2) 전용 드론 제어 시스템
Jetson Orin Nano -> UART -> FC H743v2 (ArduPilot)
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
        self.current_command = ["level", "hover", 0, 0]
        self.is_armed = False
        self.control_thread = None
        self.running = False
        
        # 쓰로틀 설정 (ArduPilot PWM)
        self.THROTTLE_TAKEOFF = 1700  # 90%
        self.THROTTLE_HOVER = 1500    # 60%
        self.THROTTLE_LAND = 1400      # 50%
        
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
            return True
        else:
            print("시동 실패")
            return False
    
    def disarm(self) -> bool:
        """시동 끄기"""
        print("시동 끄기...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0=disarm
            0, 0, 0, 0, 0, 0
        )
        
        self.is_armed = False
        time.sleep(1)
        print("시동 꺼짐")
        return True
    
    def set_command(self, vertical: str, horizontal: str, rotation: int, speed: int):
        """
        4가지 데이터로 드론 제어
        
        Args:
            vertical: "up", "level", "down"
            horizontal: "forward", "backward", "left", "right", 
                       "forward_left", "forward_right", 
                       "backward_left", "backward_right", "hover"
            rotation: 0-359 (시계방향 회전)
            speed: 0-100 (모터 부하 %)
        """
        if not self.is_armed:
            print("시동이 걸려있지 않음")
            return False
        
        self.current_command = [vertical, horizontal, rotation, speed]
        print(f"명령: [{vertical}, {horizontal}, {rotation}°, {speed}%]")
        return True
    
    def _control_loop(self):
        """제어 루프 (백그라운드)"""
        while self.running:
            try:
                if self.is_armed:
                    vertical, horizontal, rotation, speed = self.current_command
                    
                    # PWM 값 계산 (1000-2000)
                    throttle_pwm = 1000  # 기본값
                    roll_pwm = 1500      # 중립
                    pitch_pwm = 1500     # 중립
                    yaw_pwm = 1500       # 중립
                    
                    # Vertical (쓰로틀)
                    if vertical == "up":
                        throttle_pwm = self.THROTTLE_TAKEOFF
                    elif vertical == "down":
                        throttle_pwm = self.THROTTLE_LAND
                    else:  # level
                        throttle_pwm = self.THROTTLE_HOVER
                    
                    # 속도 보정
                    throttle_pwm = 1000 + int((throttle_pwm - 1000) * (speed / 100.0))
                    
                    # Horizontal (이동)
                    move_amount = int(300 * (speed / 100.0))  # 최대 ±300 PWM
                    
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
    
    def goto_gps(self, latitude: float, longitude: float, altitude: Optional[float] = None):
        """GPS 좌표로 이동 (GPS 모듈 필요)"""
        if not self.is_armed:
            return False
        
        # GUIDED 모드로 전환
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED = 4
        )
        time.sleep(1)
        
        # 목표 위치 전송
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude if altitude else 10,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        print(f"GPS 이동: ({latitude:.6f}, {longitude:.6f})")
        return True
    
    def brake(self):
        """긴급 정지 / 호버링"""
        print("브레이크! (호버링)")
        
        # LOITER 모드 (ArduPilot 호버링)
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            5  # LOITER = 5
        )
        
        # 수동으로도 중립 설정
        self.current_command = ["level", "hover", 0, 0]
        time.sleep(0.5)
        
        # STABILIZE 모드로 복귀
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0  # STABILIZE = 0
        )
        
        print("호버링 중...")
    
    def close(self):
        """연결 종료"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1)
        if self.master:
            self.master.close()


# ============= 사용 예제 =============

def main():
    # 드론 컨트롤러 생성
    drone = ArduPilotDroneController('/dev/ttyTHS1', 115200)
    
    if not drone.connect():
        return
    
    try:
        # 시동
        drone.arm()
        time.sleep(2)
        
        print("\n=== 모터 테스트 ===")
        
        # 쓰로틀 테스트 (지상)
        print("쓰로틀 50% (3초)...")
        drone.set_command("level", "hover", 0, 50)
        time.sleep(3)
        
        print("쓰로틀 80% (3초)...")
        drone.set_command("level", "hover", 0, 80)
        time.sleep(3)
        
        # 브레이크
        drone.brake()
        time.sleep(2)
        
        # 시동 끄기
        drone.disarm()
        
    except KeyboardInterrupt:
        print("\n긴급 중단!")
        drone.brake()
        drone.disarm()
    
    finally:
        drone.close()

if __name__ == "__main__":
    main()
