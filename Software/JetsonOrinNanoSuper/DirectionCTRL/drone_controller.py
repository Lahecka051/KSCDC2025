"""
drone_controller_shared.py
다른 모듈에서 공유 가능한 드론 컨트롤러
싱글톤 패턴으로 하나의 드론 인스턴스만 생성
"""

from pymavlink import mavutil
import time
import math
import threading
import atexit

class DroneCommandController:
    """원본 DroneCommandController 클래스"""
    
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
        
        # 속도 설정
        self.VERTICAL_SPEED = 0.5
        self.HORIZONTAL_SPEED = 1.0
        self.ROTATION_SPEED = 30.0
        
        # 거리 제어
        self.distance_control_active = False
        self.start_position = None
        self.target_distance = 0
        self.movement_direction = None
        self.distance_thread = None
        
        # 제어 스레드
        self.control_thread = None
        self.control_active = False
        self.current_command = ["level", "stay", 0, 0]
        
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
    
    def emergency_hover(self, reason="외부 요청"):
        """긴급 호버링"""
        if not self.is_armed:
            print("[드론] ⚠️ 시동이 꺼져있음")
            return False
            
        print("\n" + "="*60)
        print(f"🚨 긴급 호버링 - {reason}")
        print("="*60)
        
        self.emergency_hover_active = True
        self.distance_control_active = False
        self.control_active = False
        self.current_command = ["level", "stay", 0, 0]
        
        for _ in range(5):
            self._send_velocity_command(0, 0, 0)
            time.sleep(0.05)
        
        try:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                5  # LOITER
            )
            print("[드론] ✅ LOITER 모드")
        except:
            try:
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    2  # ALT_HOLD
                )
                print("[드론] ✅ ALT_HOLD 모드")
            except:
                pass
        
        print(f"📍 호버링 위치:")
        print(f"   고도: {self.current_altitude:.2f}m")
        print(f"   GPS: ({self.current_lat:.6f}, {self.current_lon:.6f})")
        
        time.sleep(2)
        self.emergency_hover_active = False
        
        print("[드론] ✅ 긴급 호버링 완료")
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
                self._reset_position()
                return True
            time.sleep(1)
        
        print("[드론] ❌ 시동 실패")
        return False
    
    def _reset_position(self):
        """위치 초기화"""
        time.sleep(1)
        self.start_position = (self.position_x, self.position_y, self.position_z)
        print(f"[드론] 위치 초기화: ({self.position_x:.2f}, {self.position_y:.2f})")
    
    def disarm(self):
        """시동 끄기"""
        print("[드론] 시동 끄기...")
        
        self.stop_control()
        self.stop_distance_control()
        
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
        self._reset_position()
        return True
    
    def set_command(self, vertical="level", horizontal="stay", rotation=0, distance=0):
        """이동 명령"""
        if self.emergency_hover_active:
            print("[드론] ⚠️ 긴급 호버링 중 - 명령 무시")
            return False
        
        if not self.is_armed:
            print("[드론] ⚠️ 시동이 꺼져있음")
            return False
        
        self.current_command = [vertical, horizontal, rotation, distance]
        
        if distance > 0 and horizontal != "stay":
            self._start_distance_control(vertical, horizontal, rotation, distance)
        else:
            self.stop_distance_control()
            
            if not self.control_active:
                self.start_control()
            
            vx, vy, vz = self._calculate_velocity(vertical, horizontal)
            self._send_velocity_command(vx, vy, vz)
            
            if rotation != 0:
                self._send_rotation_command(rotation)
        
        print(f"[드론] 명령: [{vertical}, {horizontal}, {rotation}°, {distance}m]")
        return True
    
    def land(self):
        """착륙"""
        print("[드론] 착륙 중...")
        
        self.stop_distance_control()
        
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
    
    # === Private 메서드들 ===
    
    def _start_distance_control(self, vertical, horizontal, rotation, distance):
        if self.emergency_hover_active:
            return
        
        self.stop_control()
        self.stop_distance_control()
        
        self.start_position = (self.position_x, self.position_y, self.position_z)
        self.target_distance = distance
        self.movement_direction = (vertical, horizontal, rotation)
        self.distance_control_active = True
        
        print(f"[드론] 거리 제어: {distance}m 이동 시작")
        
        self.distance_thread = threading.Thread(target=self._distance_control_loop, daemon=True)
        self.distance_thread.start()
    
    def _distance_control_loop(self):
        vertical, horizontal, rotation = self.movement_direction
        
        if rotation != 0:
            self._send_rotation_command(rotation)
            time.sleep(2)
        
        while self.distance_control_active and not self.emergency_hover_active:
            moved_distance = self._calculate_moved_distance()
            remaining = self.target_distance - moved_distance
            
            print(f"  이동: {moved_distance:.2f}m / {self.target_distance}m", end='\r')
            
            if moved_distance >= self.target_distance * 0.95:
                print(f"\n[드론] ✅ 목표 도달: {moved_distance:.2f}m")
                self.stop_distance_control()
                self._send_velocity_command(0, 0, 0)
                break
            
            speed_factor = remaining / 0.5 if remaining < 0.5 else 1.0
            
            vx, vy, vz = self._calculate_velocity(vertical, horizontal)
            self._send_velocity_command(vx * speed_factor, vy * speed_factor, vz * speed_factor)
            
            time.sleep(0.1)
    
    def _calculate_moved_distance(self):
        if not self.start_position:
            return 0
        
        dx = self.position_x - self.start_position[0]
        dy = self.position_y - self.start_position[1]
        
        return math.sqrt(dx**2 + dy**2)
    
    def stop_distance_control(self):
        self.distance_control_active = False
        if self.distance_thread:
            self.distance_thread.join(timeout=1)
    
    def _calculate_velocity(self, vertical, horizontal):
        vz = 0
        if vertical == "up":
            vz = -self.VERTICAL_SPEED
        elif vertical == "down":
            vz = self.VERTICAL_SPEED
        
        vx, vy = 0, 0
        
        velocity_map = {
            "forward": (self.HORIZONTAL_SPEED, 0),
            "backward": (-self.HORIZONTAL_SPEED, 0),
            "left": (0, -self.HORIZONTAL_SPEED),
            "right": (0, self.HORIZONTAL_SPEED),
            "forward_left": (self.HORIZONTAL_SPEED * 0.707, -self.HORIZONTAL_SPEED * 0.707),
            "forward_right": (self.HORIZONTAL_SPEED * 0.707, self.HORIZONTAL_SPEED * 0.707),
            "backward_left": (-self.HORIZONTAL_SPEED * 0.707, -self.HORIZONTAL_SPEED * 0.707),
            "backward_right": (-self.HORIZONTAL_SPEED * 0.707, self.HORIZONTAL_SPEED * 0.707),
        }
        
        if horizontal in velocity_map:
            vx, vy = velocity_map[horizontal]
        
        return vx, vy, vz
    
    def _send_velocity_command(self, vx, vy, vz):
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
    
    def _send_rotation_command(self, angle):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle,
            self.ROTATION_SPEED,
            1, 1,
            0, 0, 0
        )
    
    def start_control(self):
        self.control_active = True
        
        def control_loop():
            while self.control_active:
                if self.emergency_hover_active or self.distance_control_active:
                    time.sleep(0.2)
                    continue
                
                vertical, horizontal, rotation, _ = self.current_command
                
                if horizontal != "stay" or vertical != "level":
                    vx, vy, vz = self._calculate_velocity(vertical, horizontal)
                    self._send_velocity_command(vx, vy, vz)
                
                time.sleep(0.2)
        
        self.control_thread = threading.Thread(target=control_loop, daemon=True)
        self.control_thread.start()
    
    def stop_control(self):
        self.control_active = False
        if self.control_thread:
            self.control_thread.join(timeout=1)


# ========== 싱글톤 드론 매니저 ==========

class DroneManager:
    """싱글톤 패턴으로 드론 인스턴스 관리"""
    
    _instance = None
    _drone = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DroneManager, cls).__new__(cls)
        return cls._instance
    
    def get_drone(self):
        """드론 인스턴스 반환 (없으면 생성)"""
        if self._drone is None:
            self._drone = DroneCommandController()
            self._drone.connect()
            
            # 프로그램 종료시 자동 정리
            atexit.register(self._cleanup)
            
        return self._drone
    
    def _cleanup(self):
        """프로그램 종료시 정리"""
        if self._drone and self._drone.is_armed:
            print("\n[매니저] 프로그램 종료 - 안전 착륙")
            try:
                self._drone.land()
                self._drone.disarm()
            except:
                pass


# 전역 드론 인스턴스 생성 함수
def get_drone():
    """전역 드론 인스턴스 반환"""
    manager = DroneManager()
    return manager.get_drone()
