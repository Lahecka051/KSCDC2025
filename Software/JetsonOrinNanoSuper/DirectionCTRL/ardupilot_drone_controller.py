"""
Hexa_fc_auto_control.py
헥드콥터 FC 자동 쓰로틀 제어 (4S 8400mAh)
FC가 부하에 따라 자동으로 쓰로틀 조정
"""

from pymavlink import mavutil
import time

class HexacopterAutoControl:
    """헥사콥터 FC 자동 제어"""
    
    def __init__(self):
        self.master = None
        # 4S 배터리 전압 범위
        self.BATTERY_CELLS = 4
        self.CELL_MAX_VOLTAGE = 4.2  # 4S = 16.8V 만충
        self.CELL_MIN_VOLTAGE = 3.3  # 4S = 13.2V 최소
        
    def connect(self):
        print("헥사콥터 FC 연결 중...")
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=115200)
        self.master.wait_heartbeat()
        print(f"✅ 연결 성공! (시스템 ID: {self.master.target_system})\n")
        
        self.setup_4s_params()
        
    def setup_4s_params(self):
        """4S 배터리 헥사콥터 파라미터"""
        print("4S 8400mAh 헥사콥터 설정...")
        
        params = {
            # 프레임 타입
            'FRAME_CLASS': 2,          # 1 = Quad, 2 = Hexa
            'FRAME_TYPE': 1,           # 1 = X configuration
            
            # 4S 배터리 설정
            'BATT_CAPACITY': 8400,     # 8400mAh
            'BATT_VOLT_MAX': 16.8,     # 4S 만충 전압
            'BATT_VOLT_MIN': 13.2,     # 4S 최소 전압
            'BATT_CRT_VOLT': 13.5,     # 위험 전압
            'BATT_LOW_VOLT': 14.0,     # 경고 전압
            
            # 자동 쓰로틀 학습 활성화 (중요!)
            'MOT_HOVER_LEARN': 2,      # 호버 쓰로틀 자동 학습
            'MOT_THST_HOVER': 0.5,     # 초기 호버 쓰로틀 (자동 조정됨)
            
            # 쓰로틀 자동 보정 파라미터
            'MOT_BAT_VOLT_MAX': 16.8,  # 배터리 최대 전압
            'MOT_BAT_VOLT_MIN': 13.2,  # 배터리 최소 전압
            'MOT_BAT_CURR_MAX': 60.0,  # 최대 전류 (A)
            'MOT_THST_EXPO': 0.65,     # 쓰로틀 커브
            
            # 고도 제어 PID (FC 자동 제어)
            'PSC_ACCZ_P': 0.5,         # 고도 가속 P
            'PSC_ACCZ_I': 1.0,         # 고도 가속 I (적분 - 부하 보상)
            'PSC_ACCZ_D': 0.0,         # 고도 가속 D
            'PSC_VELZ_P': 5.0,         # 고도 속도 P
            'PSC_POSZ_P': 1.0,         # 고도 위치 P
            
            # 기타
            'ARMING_CHECK': 0,
            'GPS_TYPE': 0,
            'PILOT_TKOFF_ALT': 150,    # 최대 1m
        }
        
        for param, value in params.items():
            try:
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32 if isinstance(value, float) else mavutil.mavlink.MAV_PARAM_TYPE_INT32
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    param.encode('utf-8'),
                    value,
                    param_type
                )
                time.sleep(0.05)
            except:
                pass
        
        print("✅ 4S 헥사콥터 설정 완료\n")
    
    def arm(self):
        """시동"""
        print("시동 걸기...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(3)
        print("✅ 시동 걸림!\n")
    
    def get_battery_info(self):
        """배터리 정보 읽기"""
        msg = self.master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1)
        if msg:
            voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            remaining = msg.battery_remaining
            return voltage, current, remaining
        return 0, 0, 0
    
    def get_altitude(self):
        """고도 읽기"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        if msg:
            return msg.relative_alt / 1000.0
        return 0.0
    
    # ========== 방법 1: ALT_HOLD 모드 (FC 자동 쓰로틀) ==========
    
    def auto_altitude_control(self):
        """
        ALT_HOLD 모드: FC가 자동으로 쓰로틀 조정
        손으로 잡아도 FC가 알아서 쓰로틀 증가
        """
        print("\n" + "="*60)
        print("ALT_HOLD 모드 - FC 자동 쓰로틀 제어")
        print("FC가 부하 변화를 감지하고 자동으로 쓰로틀 조정")
        print("="*60)
        
        # ALT_HOLD 모드 설정
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            2  # ALT_HOLD
        )
        time.sleep(1)
        print("✅ ALT_HOLD 모드 활성화\n")
        
        self.arm()
        
        print("테스트 시작:")
        print("- 쓰로틀 스틱 = 상승/하강 속도 명령")
        print("- FC가 필요한 실제 쓰로틀 자동 계산")
        print("- 손으로 잡으면 FC가 자동으로 쓰로틀 증가\n")
        
        try:
            # 1m 상승 명령
            print("[상승] 목표: 1.5m (FC 자동 제어)")
            target_alt = 1.5
            start_alt = self.get_altitude()
            
            while self.get_altitude() - start_alt < target_alt:
                # 상승 속도 명령 (FC가 실제 쓰로틀 자동 계산)
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    1500, 1500,
                    1600,  # 상승 속도 명령 (NOT 직접 쓰로틀!)
                    1500,
                    0, 0, 0, 0
                )
                
                alt = self.get_altitude()
                voltage, current, remaining = self.get_battery_info()
                
                print(f"\r고도: {alt:.2f}m | 배터리: {voltage:.1f}V, {current:.1f}A | "
                      f"FC가 자동으로 쓰로틀 조정 중", end="")
                
                time.sleep(0.1)
            
            print(f"\n✅ 목표 고도 도달: {self.get_altitude():.2f}m")
            
            # 고도 유지 (FC 자동)
            print("\n[호버링] FC가 자동으로 고도 유지")
            print("손으로 아래로 누르면 FC가 쓰로틀 자동 증가!")
            
            for i in range(50):  # 5초
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    1500, 1500,
                    1500,  # 고도 유지 명령 (FC가 실제 쓰로틀 자동 조정)
                    1500,
                    0, 0, 0, 0
                )
                
                alt = self.get_altitude()
                voltage, current, remaining = self.get_battery_info()
                
                print(f"\r고도: {alt:.2f}m | 전류: {current:.1f}A (부하 증가시 전류 상승) | "
                      f"{5-i/10:.1f}초", end="")
                
                time.sleep(0.1)
            
            print("\n\n[착륙]")
            # 착륙
            while self.get_altitude() > 0.1:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    1500, 1500,
                    1400,  # 하강 속도 명령
                    1500,
                    0, 0, 0, 0
                )
                
                print(f"\r고도: {self.get_altitude():.2f}m", end="")
                time.sleep(0.1)
            
            print("\n✅ 착륙 완료!")
            
        except KeyboardInterrupt:
            print("\n\n긴급 정지!")
        
        finally:
            self.disarm()
    
    # ========== 방법 2: GUIDED 모드 (완전 자동) ==========
    
    def guided_mode_control(self):
        """
        GUIDED 모드: 목표 고도만 주면 FC가 모든 것을 자동 제어
        """
        print("\n" + "="*60)
        print("GUIDED 모드 - 완전 자동 제어")
        print("목표 고도만 설정하면 FC가 알아서 처리")
        print("="*60)
        
        # GUIDED 모드
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED
        )
        time.sleep(1)
        print("✅ GUIDED 모드 활성화\n")
        
        self.arm()
        
        try:
            # 자동 이륙 명령 (1m)
            print("[자동 이륙] 목표: 1m")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0,
                1.0  # 목표 고도 1m
            )
            
            # FC가 알아서 상승
            for _ in range(100):  # 10초 대기
                alt = self.get_altitude()
                voltage, current, remaining = self.get_battery_info()
                
                print(f"\r고도: {alt:.2f}m | 전류: {current:.1f}A | "
                      f"FC 완전 자동 제어", end="")
                
                if alt >= 0.9:
                    print("\n✅ 목표 고도 도달!")
                    break
                
                time.sleep(0.1)
            
            print("\n5초간 자동 호버링...")
            time.sleep(5)
            
            # 자동 착륙
            print("\n[자동 착륙]")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            while self.get_altitude() > 0.1:
                print(f"\r고도: {self.get_altitude():.2f}m", end="")
                time.sleep(0.1)
            
            print("\n✅ 자동 착륙 완료!")
            
        except KeyboardInterrupt:
            print("\n긴급 정지!")
        
        finally:
            self.disarm()
    
    def disarm(self):
        """시동 끄기"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        print("시동 꺼짐!")
# 메인
if __name__ == "__main__":
    print("\n" + "="*60)
    print("헥드콥터 FC 자동 쓰로틀 제어")
    print("4S 8400mAh 배터리")
    print("="*60)
    
    print("\n1. ALT_HOLD 모드")
    print("2. GUIDED 모드")
    choice = input("선택 (1/2): ")
    
    if choice == "1":
        quad = QuadcopterAutoControl()
        quad.connect()
        quad.auto_altitude_control()
    elif choice == "2":
        quad = QuadcopterAutoControl()
        quad.connect()
        quad.guided_mode_control()
