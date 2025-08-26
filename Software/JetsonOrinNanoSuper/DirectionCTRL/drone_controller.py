"""
drone_controller.py
드론 컨트롤러 - 수치 기반 속도 제어
"""

from pymavlink import mavutil
import time
import math
import threading

class DroneCommandController:
    """드론 컨트롤러 클래스"""
    
    def __init__(self, connection_string='/dev/ttyTHS1', baudrate=115200):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        
        # 현재 상태
        self.is_armed = False
        self.is_connected = False
        self.current_altitude = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        
        # EKF 기반 위치
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        # 속도 제한
        self.MAX_SPEED = 10.0  # 최대 속도 10m/s
        
        # 제어 스레드
        self.control_thread = None
        self.control_active = False
        self.current_command = [0, 0, 0, 0]  # [vertical, horizontal1, horizontal2, rotation]
        
        # 긴급 호버링
        self.emergency_hover_active = False
        
    def connect(self):
        """FC 연결"""
        if self.is_connected:
            print("[드론] 이미 연결되어 있습니다")
            return True
            
        try:
            print("[드론] FC 연결 중...")
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            self.master.wait_heartbeat()
            print(f"[드론] ✅ FC 연결 성공")
            
            self._wait_for_ekf()
            self.start_monitoring()
            self.is_connected = True
            return True
        except Exception as e:
            print(f"[드론] ❌ 연결 실패: {e}")
            return False
    
    def _wait_for_ekf(self):
        """EKF 초기화 대기"""
        print("[드론] EKF 초기화 중...")
        
        for _ in range(30):
            msg = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
            if msg and msg.flags & 0x01:
                print("[드론] ✅ EKF 준비 완료")
                return True
            time.sleep(0.5)
        
        print("[드론] ⚠️ EKF 초기화 지연")
        return False
    
    def start_monitoring(self):
        """백그라운드 모니터링"""
        def monitor():
            while self.master and self.is_connected:
                try:
                    msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                    if msg:
                        self.position_x = msg.x
                        self.position_y = msg.y
                        self.position_z = msg.z
                    
                    msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if msg:
                        self.current_altitude = msg.relative_alt / 1000.0
                        self.current_lat = msg.lat / 1e7
                        self.current_lon = msg.lon / 1e7
                        self.current_heading = msg.hdg / 100.0
                    
                    msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
                    if msg:
                        self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    
                    time.sleep(0.05)
                except:
                    pass
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
    
    def set_command(self, vertical=0, horizontal1=0, horizontal2=0, rotation=0):
        """
        이동 명령 설정
        
        Args:
            vertical: 수직 속도 (m/s) - 양수:상승, 음수:하강, 0:유지 (최대 ±10)
            horizontal1: 전후 속도 (m/s) - 양수:전진, 음수:후진, 0:유지 (최대 ±10)
            horizontal2: 좌우 속도 (m/s) - 양수:좌측, 음수:우측, 0:유지 (최대 ±10)
            rotation: 회전 각도 (0-359도) - 시계방향
        
        Examples:
            drone.set_command(1.0, 0, 0, 0)      # 1m/s 상승
            drone.set_command(-0.5, 0, 0, 0)     # 0.5m/s 하강
            drone.set_command(0, 2.0, 0, 0)      # 2m/s 전진
            drone.set_command(0, -1.0, 0, 0)     # 1m/s 후진
            drone.set_command(0, 0, 1.5, 0)      # 1.5m/s 좌측
            drone.set_command(0, 0, -1.5, 0)     # 1.5m/s 우측
            drone.set_command(0, 1.0, 1.0, 0)    # 전진+좌측 대각선
            drone.set_command(0, 0, 0, 90)       # 90도 회전
            drone.set_command(0, 0, 0, 0)        # 호버링
        """
        
        if self.emergency_hover_active:
            print("[드론] ⚠️ 긴급 호버링 중 - 명령 무시")
            return False
        
        if not self.is_armed:
            print("[드론] ⚠️ 시동이 꺼져있음")
            return False
        
        # 속도 제한 (최대 10m/s)
        vertical = max(-self.MAX_SPEED, min(self.MAX_SPEED, vertical))
        horizontal1 = max(-self.MAX_SPEED, min(self.MAX_SPEED, horizontal1))
        horizontal2 = max(-self.MAX_SPEED, min(self.MAX_SPEED, horizontal2))
        
        # 회전 각도 정규화 (0-359)
        rotation = rotation % 360
        
        # 명령 저장
        self.current_command = [vertical, horizontal1, horizontal2, rotation]
        
        # 제어 루프 시작
        if not self.control_active:
            self.start_control()
        
        # 속도 명령 전송
        self._send_velocity_command(vertical, horizontal1, horizontal2)
        
        # 회전 명령
        if rotation != 0:
            self._send_rotation_command(rotation)
        
        # 상태 출력
        status = self._get_command_description(vertical, horizontal1, horizontal2, rotation)
        print(f"[드론] 명령: {status}")
        print(f"        [V:{vertical:+.1f}, H1:{horizontal1:+.1f}, H2:{horizontal2:+.1f}, R:{rotation}°]")
        
        return True
    
    def _get_command_description(self, v, h1, h2, r):
        """명령 설명 생성"""
        parts = []
        
        # 수직
        if v > 0:
            parts.append(f"상승 {v:.1f}m/s")
        elif v < 0:
            parts.append(f"하강 {abs(v):.1f}m/s")
        
        # 전후
        if h1 > 0:
            parts.append(f"전진 {h1:.1f}m/s")
        elif h1 < 0:
            parts.append(f"후진 {abs(h1):.1f}m/s")
        
        # 좌우
        if h2 > 0:
            parts.append(f"좌측 {h2:.1f}m/s")
        elif h2 < 0:
            parts.append(f"우측 {abs(h2):.1f}m/s")
        
        # 회전
        if r != 0:
            parts.append(f"회전 {r}°")
        
        # 호버링
        if v == 0 and h1 == 0 and h2 == 0 and r == 0:
            return "호버링"
        
        return " + ".join(parts) if parts else "유지"
    
    def _send_velocity_command(self, vertical, horizontal1, horizontal2):
        """속도 명령 전송"""
        # NED 좌표계 변환
        # North (전진+), East (우측+), Down (하강+)
        vx = horizontal1   # 전진(+) / 후진(-)
        vy = -horizontal2  # 좌측(+) → East(-) / 우측(-) → East(+)
        vz = -vertical     # 상승(+) → Down(-) / 하강(-) → Down(+)
        
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 드론 기준 좌표
            0b0000111111000111,  # 속도만 제어
            0, 0, 0,  # 위치 (사용 안함)
            vx, vy, vz,  # 속도 (m/s)
            0, 0, 0,  # 가속도 (사용 안함)
            0, 0  # yaw, yaw_rate
        )
    
    def _send_rotation_command(self, angle):
        """회전 명령 전송"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle,  # 목표 각도
            30.0,   # 회전 속도 (도/초)
            1,      # 시계방향
            1,      # 상대 각도
            0, 0, 0
        )
    
    def emergency_hover(self, reason="외부 요청"):
        """긴급 호버링 (GUIDED 모드 유지)"""
        if not self.is_armed:
            print("[드론] ⚠️ 시동이 꺼져있음")
            return False
            
        print("\n" + "="*60)
        print(f"🚨 긴급 호버링 - {reason}")
        print("="*60)
        
        # 긴급 호버링 활성화
        self.emergency_hover_active = True
        
        # 현재 명령 취소 (속도 0으로 설정)
        self.current_command = [0, 0, 0, 0]
        
        # 즉시 정지 명령 전송 (GUIDED 모드 유지)
        print("[드론] GUIDED 모드 유지하며 속도 0 설정")
        for _ in range(10):  # 여러 번 전송하여 확실하게 정지
            self._send_velocity_command(0, 0, 0)
            time.sleep(0.05)
        
        print("[드론] ✅ 속도 명령 취소 - 호버링 중")
        
        # 현재 위치 출력
        print(f"📍 호버링 위치:")
        print(f"   고도: {self.current_altitude:.2f}m")
        print(f"   GPS: ({self.current_lat:.6f}, {self.current_lon:.6f})")
        print(f"   모드: GUIDED (유지)")
        
        # 잠시 대기
        time.sleep(2)
        
        # 긴급 호버링 해제 (다시 명령 받을 준비)
        self.emergency_hover_active = False
        
        print("[드론] ✅ 긴급 호버링 완료 - 새 명령 대기")
        return True
    
    def arm(self):
        """시동"""
        print("[드론] 시동 걸기...")
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED
        )
        time.sleep(1)
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        for _ in range(10):
            if self.is_armed:
                print("[드론] ✅ 시동 걸림")
                return True
            time.sleep(1)
        
        print("[드론] ❌ 시동 실패")
        return False
    
    def disarm(self):
        """시동 끄기"""
        print("[드론] 시동 끄기...")
        
        self.control_active = False
        if self.control_thread:
            self.control_thread.join(timeout=1)
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        
        time.sleep(2)
        print("[드론] ✅ 시동 꺼짐")
    
    def takeoff(self, altitude=1.5):
        """이륙"""
        print(f"[드론] 이륙 (목표: {altitude}m)...")
        
        if not self.is_armed:
            if not self.arm():
                return False
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0,
            altitude
        )
        
        while self.current_altitude < altitude * 0.9:
            print(f"[드론] 고도: {self.current_altitude:.2f}m", end='\r')
            time.sleep(0.5)
        
        print(f"\n[드론] ✅ 이륙 완료: {self.current_altitude:.2f}m")
        return True
    
    def land(self):
        """착륙"""
        print("[드론] 착륙 중...")
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9  # LAND
        )
        
        while self.current_altitude > 0.1:
            print(f"[드론] 고도: {self.current_altitude:.2f}m", end='\r')
            time.sleep(0.5)
        
        print("\n[드론] ✅ 착륙 완료")
    
    def get_status(self):
        """현재 상태 반환"""
        return {
            'connected': self.is_connected,
            'armed': self.is_armed,
            'altitude': self.current_altitude,
            'gps': (self.current_lat, self.current_lon),
            'heading': self.current_heading,
            'position': (self.position_x, self.position_y, self.position_z),
            'emergency': self.emergency_hover_active,
            'command': self.current_command
        }
    
    def start_control(self):
        """제어 루프 시작"""
        self.control_active = True
        
        def control_loop():
            while self.control_active:
                if self.emergency_hover_active:
                    # 긴급 호버링 중에는 계속 속도 0 전송
                    self._send_velocity_command(0, 0, 0)
                    time.sleep(0.1)
                    continue
                
                v, h1, h2, r = self.current_command
                
                # 속도 명령 지속 전송
                if v != 0 or h1 != 0 or h2 != 0:
                    self._send_velocity_command(v, h1, h2)
                
                time.sleep(0.1)  # 10Hz
        
        self.control_thread = threading.Thread(target=control_loop, daemon=True)
        self.control_thread.start()


# 테스트 코드
if __name__ == "__main__":
    drone = DroneCommandController()
    drone.connect()
    
    # 테스트 예시
    drone.takeoff(2.0)
    time.sleep(2)
    
    # 전진 중
    drone.set_command(0, 2.0, 0, 0)
    time.sleep(2)
    
    # 긴급 호버링 (GUIDED 모드 유지)
    drone.emergency_hover("테스트")
    time.sleep(3)
    
    # 다시 명령 가능
    drone.set_command(0, 1.0, 0, 0)
    time.sleep(2)
    
    drone.land()
    drone.disarm()
