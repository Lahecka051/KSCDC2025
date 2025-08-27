
"""
drone_controller.py
드론 컨트롤러 - FC 자동 설정 및 최단 경로 회전
"""

from pymavlink import mavutil
import time
import math
import threading

class DroneCommandController:
    """드론 컨트롤러 클래스"""
    
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=115200):
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
        
        # 위치 정보
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        
        # 고도 정보 (다중 소스)
        self.vfr_altitude = 0.0      # VFR_HUD 기압계 고도 (최우선)
        self.ned_altitude = 0.0      # LOCAL_POSITION_NED 고도
        self.relative_altitude = 0.0 # GLOBAL_POSITION_INT 상대고도
        
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
            
            # 최대 샘플링 주파수로 데이터 스트림 요청
            self.master.mav.request_data_stream_send(
                self.master.target_system,      # param1: 타겟 시스템 ID (FC)
                self.master.target_component,   # param2: 타겟 컴포넌트 ID
                mavutil.mavlink.MAV_DATA_STREAM_ALL,  # param3: 스트림 타입 (모든 데이터)
                50,  # param4: 전송률 50Hz (최대 샘플링)
                1    # param5: 1=활성화, 0=비활성화
            )
            time.sleep(1)  # 1초 대기
            
            self.start_monitoring()
            self.is_connected = True
            return True
        except Exception as e:
            print(f"[드론] ❌ 연결 실패: {e}")
            return False
    
    def get_reliable_altitude(self):
        """가장 신뢰할 수 있는 고도 반환"""
        # 1순위: VFR_HUD 기압계 고도
        if self.vfr_altitude is not None:
            return self.vfr_altitude, 'VFR_HUD'
        
        # 2순위: LOCAL_POSITION_NED
        if self.ned_altitude is not None:
            return self.ned_altitude, 'LOCAL_NED'
        
        # 3순위: GLOBAL_POSITION_INT 상대고도
        if self.relative_altitude is not None:
            return self.relative_altitude, 'GLOBAL_REL'
        
        return 0.0, 'NONE'
    
    def start_monitoring(self):
        """백그라운드 모니터링 (50Hz 샘플링)"""
        def monitor():
            while self.master and self.is_connected:
                try:
                    # VFR_HUD - 기압계 고도 (최우선)
                    msg = self.master.recv_match(type='VFR_HUD', blocking=False)
                    if msg:
                        self.vfr_altitude = msg.alt
                        self.current_altitude = msg.alt  # 기본 고도를 VFR로 설정
                    
                    # LOCAL_POSITION_NED
                    msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                    if msg:
                        self.position_x = msg.x
                        self.position_y = msg.y
                        self.position_z = msg.z
                        self.ned_altitude = -msg.z  # z는 음수가 위
                    
                    # GLOBAL_POSITION_INT
                    msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if msg:
                        self.relative_altitude = msg.relative_alt / 1000.0  # 밀리미터를 미터로 변환
                        self.current_lat = msg.lat / 1e7  # 1e7로 나눠서 도 단위로 변환
                        self.current_lon = msg.lon / 1e7  # 1e7로 나눠서 도 단위로 변환
                        self.current_heading = msg.hdg / 100.0  # 센티도를 도로 변환
                    
                    # HEARTBEAT
                    msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
                    if msg:
                        self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    
                    time.sleep(0.02)  # 50Hz 샘플링 (20ms = 0.02초)
                except:
                    pass
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
    
    def set_command(self, vertical=0, horizontal1=0, horizontal2=0, rotation=None):
        """
        이동 명령 설정 (속도 제한 없음)
        
        Args:
            vertical: 수직 속도 (m/s) - 양수:상승, 음수:하강, 0:유지
            horizontal1: 전후 속도 (m/s) - 양수:전진, 음수:후진, 0:유지
            horizontal2: 좌우 속도 (m/s) - 양수:좌측, 음수:우측, 0:유지
            rotation: 목표 각도 (0-359도) - None이면 회전 안함, 값이 있으면 최단경로 회전
        
        Examples:
            drone.set_command(1.0, 0, 0)         # 1m/s 상승
            drone.set_command(-0.5, 0, 0)        # 0.5m/s 하강
            drone.set_command(0, 2.0, 0)         # 2m/s 전진
            drone.set_command(0, -1.0, 0)        # 1m/s 후진
            drone.set_command(0, 0, 1.5)         # 1.5m/s 좌측
            drone.set_command(0, 0, -1.5)        # 1.5m/s 우측
            drone.set_command(0, 1.0, 1.0)       # 전진+좌측 대각선
            drone.set_command(0, 0, 0, 90)       # 90도로 회전 (최단 경로)
            drone.set_command(0, 0, 0, 270)      # 270도로 회전 (최단 경로)
            drone.set_command(0, 0, 0, 0)        # 호버링
        """
        
        if self.emergency_hover_active:
            print("[드론] ⚠️ 긴급 호버링 중 - 명령 무시")
            return False
        
        if not self.is_armed:
            print("[드론] ⚠️ 시동이 꺼져있음")
            return False
        
        # 회전값 처리
        if rotation is not None:
            rotation = rotation % 360  # 0-359 정규화
        
        # 명령 저장
        self.current_command = [vertical, horizontal1, horizontal2, rotation if rotation is not None else 0]
        
        # 제어 루프 시작
        if not self.control_active:
            self.start_control()
        
        # 속도 명령 전송
        self._send_velocity_command(vertical, horizontal1, horizontal2)
        
        # 회전 명령 (최단 경로 자동 선택)
        if rotation is not None:
            self._send_rotation_command(rotation)
        
        # 상태 출력 (고도 소스 포함)
        alt, source = self.get_reliable_altitude()
        status = self._get_command_description(vertical, horizontal1, horizontal2, rotation)
        print(f"[드론] 명령: {status}")
        print(f"        [V:{vertical:+.1f}, H1:{horizontal1:+.1f}, H2:{horizontal2:+.1f}, R:{rotation if rotation is not None else 'N/A'}°]")
        print(f"        고도: {alt:.2f}m [{source}], 현재방향: {self.current_heading:.0f}°")
        
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
        
        # 회전 (최단 경로 표시)
        if r is not None:
            # 최단 경로 계산
            current = self.current_heading
            diff = (r - current) % 360
            if diff > 180:
                diff -= 360
            
            if diff > 0:
                parts.append(f"{r}°로 회전 (시계방향 {abs(diff):.0f}°)")
            elif diff < 0:
                parts.append(f"{r}°로 회전 (반시계 {abs(diff):.0f}°)")
            else:
                parts.append(f"현재 방향 유지 ({r}°)")
        
        # 호버링
        if v == 0 and h1 == 0 and h2 == 0 and r is None:
            return "호버링"
        
        return " + ".join(parts) if parts else "유지"
    
    def _send_velocity_command(self, vertical, horizontal1, horizontal2):
        """속도 명령 전송 (20Hz)"""
        # NED 좌표계 변환
        vx = horizontal1   # 전진(+) / 후진(-)
        vy = -horizontal2  # 좌측(+) → East(-) / 우측(-) → East(+)
        vz = -vertical     # 상승(+) → Down(-) / 하강(-) → Down(+)
        
        self.master.mav.set_position_target_local_ned_send(
            0,                               # param1: time_boot_ms (0=무시)
            self.master.target_system,       # param2: 타겟 시스템 ID
            self.master.target_component,    # param3: 타겟 컴포넌트 ID
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # param4: 좌표계 (바디 오프셋 NED)
            0b0000111111000111,              # param5: type_mask 비트마스크
            0,                               # param6: x 위치 (미터) - 무시됨
            0,                               # param7: y 위치 (미터) - 무시됨
            0,                               # param8: z 위치 (미터) - 무시됨
            vx,                              # param9: x 속도 (m/s) - 전후
            vy,                              # param10: y 속도 (m/s) - 좌우
            vz,                              # param11: z 속도 (m/s) - 상하
            0,                               # param12: x 가속도 (m/s^2) - 무시됨
            0,                               # param13: y 가속도 (m/s^2) - 무시됨
            0,                               # param14: z 가속도 (m/s^2) - 무시됨
            0,                               # param15: yaw (라디안) - 무시됨
            0                                # param16: yaw_rate (rad/s) - 무시됨
        )
    
    def _send_rotation_command(self, angle, rotation_speed=None):
        """
        회전 명령 전송 (FC가 최단 경로 자동 선택)
        
        Args:
            angle: 목표 각도 (0-359)
            rotation_speed: 회전 속도 (도/초) - None이면 FC 자동 (보통 30도/초)
        """
        if rotation_speed is None:
            rotation_speed = 0  # 0 = FC 자동 결정
            
        self.master.mav.command_long_send(
            self.master.target_system,       # param1: 타겟 시스템 ID
            self.master.target_component,    # param2: 타겟 컴포넌트 ID
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # param3: 명령 ID (YAW 제어)
            0,                # param4: confirmation (0=첫 번째 전송)
            angle,            # param5: 목표 각도 (0-359도)
            rotation_speed,   # param6: 회전 속도 (도/초) - 0=FC 자동
            0,                # param7: 방향 (0=최단경로, 1=시계, -1=반시계)
            0,                # param8: 상대/절대 (0=절대각도, 1=상대각도)
            0,                # param9: 예약됨
            0,                # param10: 예약됨
            0                 # param11: 예약됨
        )
        
        # 최단 경로 정보 출력
        current = self.current_heading
        diff = (angle - current) % 360
        if diff > 180:
            diff -= 360
        
        if diff != 0:
            direction = "시계방향" if diff > 0 else "반시계방향"
            print(f"[드론] 회전: {current:.0f}° → {angle}° ({direction} {abs(diff):.0f}°)")
    
    def rotate_to(self, target_angle, rotation_speed=None):
        """
        특정 각도로 회전 (별도 함수)
        
        Args:
            target_angle: 목표 각도 (0-359)
            rotation_speed: 회전 속도 (도/초) - None이면 FC 자동
        
        Examples:
            drone.rotate_to(90)          # 90도로 회전 (FC 자동 속도)
            drone.rotate_to(180, 45)     # 180도로 45도/초 속도로 회전
            drone.rotate_to(0)           # 북쪽(0도)으로 회전
        """
        if not self.is_armed:
            print("[드론] ⚠️ 시동이 꺼져있음")
            return False
            
        target_angle = target_angle % 360
        self._send_rotation_command(target_angle, rotation_speed)
        return True
    
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
        
        # 즉시 정지 명령 전송 (20Hz로 여러 번)
        print("[드론] GUIDED 모드 유지하며 속도 0 설정")
        for i in range(10):  # 10번 반복
            self._send_velocity_command(0, 0, 0)  # 속도 0
            time.sleep(0.05)  # 20Hz (50ms = 0.05초)
        
        print("[드론] ✅ 속도 명령 취소 - 호버링 중")
        
        # 현재 위치 출력
        alt, source = self.get_reliable_altitude()
        print(f"📍 호버링 위치:")
        print(f"   고도: {alt:.2f}m [{source}]")
        print(f"   GPS: ({self.current_lat:.6f}, {self.current_lon:.6f})")
        print(f"   방향: {self.current_heading:.0f}°")
        print(f"   모드: GUIDED (유지)")
        
        # 잠시 대기
        time.sleep(2)  # 2초 대기
        
        # 긴급 호버링 해제 (다시 명령 받을 준비)
        self.emergency_hover_active = False
        
        print("[드론] ✅ 긴급 호버링 완료 - 새 명령 대기")
        return True
    
    def arm(self):
        """시동"""
        print("[드론] 시동 걸기...")
        
        # GUIDED 모드 설정
        self.master.mav.set_mode_send(
            self.master.target_system,       # param1: 타겟 시스템 ID
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param2: 모드 플래그
            4  # param3: 커스텀 모드 번호 (4=GUIDED)
        )
        time.sleep(1)  # 1초 대기
        
        # 시동 명령
        self.master.mav.command_long_send(
            self.master.target_system,       # param1: 타겟 시스템 ID
            self.master.target_component,    # param2: 타겟 컴포넌트 ID
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # param3: 명령 ID (시동/시동해제)
            0,  # param4: confirmation (0=첫 번째 전송)
            1,  # param5: 1=시동, 0=시동해제
            0,  # param6: 0=정상 모드, 21196=강제 시동해제
            0,  # param7: 예약됨
            0,  # param8: 예약됨
            0,  # param9: 예약됨
            0,  # param10: 예약됨
            0   # param11: 예약됨
        )
        
        # 시동 확인
        for i in range(10):  # 10번 시도 (10초)
            if self.is_armed:
                alt, source = self.get_reliable_altitude()
                print(f"[드론] ✅ 시동 걸림 (고도: {alt:.2f}m [{source}])")
                return True
            time.sleep(1)  # 1초 대기
        
        print("[드론] ❌ 시동 실패")
        return False
    
    def disarm(self):
        """시동 끄기 (정상 모드)"""
        print("[드론] 시동 끄기...")
        
        # 제어 중지
        self.control_active = False
        if self.control_thread:
            self.control_thread.join(timeout=1)  # 1초 타임아웃
        
        # 정상 시동 해제 (강제 아님)
        for i in range(3):  # 3번 시도
            self.master.mav.command_long_send(
                self.master.target_system,       # param1: 타겟 시스템 ID
                self.master.target_component,    # param2: 타겟 컴포넌트 ID
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # param3: 명령 ID
                i,  # param4: confirmation (0,1,2 재시도 카운터)
                0,  # param5: 0=시동해제, 1=시동
                0,  # param6: 0=정상 모드 (강제 아님)
                0,  # param7: 예약됨
                0,  # param8: 예약됨
                0,  # param9: 예약됨
                0,  # param10: 예약됨
                0   # param11: 예약됨
            )
            time.sleep(1)  # 1초 대기
            
            # 시동 상태 확인
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)  # 1초 타임아웃
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[드론] ✅ 시동 해제 성공")
                return True
        
        print("[드론] ⚠️ 시동 해제 실패 (수동으로 해제 필요)")
        return False
    
    def takeoff(self, altitude=None, ascent_rate=None):
        """
        이륙
        
        Args:
            altitude: 목표 고도 (미터) - None이면 FC 기본값 (보통 1.5m)
            ascent_rate: 상승률 (m/s) - None이면 FC 자동 결정
        
        Examples:
            drone.takeoff()             # FC 기본 고도, 자동 상승률
            drone.takeoff(2.0)          # 2m, FC 자동 상승률
            drone.takeoff(2.0, 0.5)     # 2m, 0.5m/s 상승률
            drone.takeoff(altitude=3)   # 3m, FC 자동 상승률
        """
        # None 처리
        if altitude is None:
            altitude = 0  # 0 = FC 기본값 (보통 1.5m)
            print(f"[드론] 이륙 (목표: FC 기본, 상승률: {'자동' if ascent_rate is None else f'{ascent_rate}m/s'})...")
        else:
            print(f"[드론] 이륙 (목표: {altitude}m, 상승률: {'자동' if ascent_rate is None else f'{ascent_rate}m/s'})...")
        
        if ascent_rate is None:
            ascent_rate = 0  # 0 = FC 자동 결정
        
        if not self.is_armed:
            if not self.arm():
                return False
        
        # 초기 고도 확인
        initial_alt, source = self.get_reliable_altitude()
        print(f"[드론] 초기 고도: {initial_alt:.2f}m [{source}]")
        
        # 이륙 명령
        self.master.mav.command_long_send(
            self.master.target_system,       # param1: 타겟 시스템 ID
            self.master.target_component,    # param2: 타겟 컴포넌트 ID
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # param3: 명령 ID (이륙)
            0,           # param4: confirmation (0=첫 번째 전송)
            0,           # param5: 피치 각도 (0=FC 자동)
            0,           # param6: 빈 값
            ascent_rate, # param7: 상승률 (m/s) - 0=FC 자동
            0,           # param8: Yaw 각도 (0=현재 유지)
            0,           # param9: 위도 (0=현재 위치)
            0,           # param10: 경도 (0=현재 위치)
            altitude     # param11: 목표 고도 (미터) - 0=FC 기본값
        )
        
        # 고도 도달 대기 (20Hz 모니터링)
        if altitude == 0:
            # FC 기본값 사용시 일정 시간 대기
            print("[드론] FC 기본 고도로 이륙 중...")
            time.sleep(5)  # 5초 대기
            current_alt, source = self.get_reliable_altitude()
            print(f"[드론] ✅ 이륙 완료: {current_alt:.2f}m")
        else:
            # 특정 고도 지정시 도달 확인
            stable_count = 0  # 안정 카운터
            while True:
                current_alt, source = self.get_reliable_altitude()
                diff = abs(current_alt - altitude)
                
                print(f"[드론] 이륙 중... 고도: {current_alt:.2f}m (목표: {altitude}m) [{source}]", end='\r')
                
                if diff < 0.2:  # 20cm 이내
                    stable_count += 1
                    if stable_count > 10:  # 10회 연속 (0.5초간) 안정
                        break
                else:
                    stable_count = 0  # 리셋
                
                time.sleep(0.05)  # 20Hz (50ms)
            
            print(f"\n[드론] ✅ 이륙 완료: {current_alt:.2f}m")
        
        return True
    
    def land(self):
        """착륙 (안전한 하강)"""
        print("[드론] 착륙 중...")
        
        # LAND 모드 전환
        self.master.mav.set_mode_send(
            self.master.target_system,       # param1: 타겟 시스템 ID
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param2: 모드 플래그
            9  # param3: 커스텀 모드 번호 (9=LAND)
        )
        
        # 착륙 모니터링 (20Hz)
        ground_detect_count = 0  # 지면 감지 카운터
        last_alt = 999  # 이전 고도
        
        while True:
            current_alt, source = self.get_reliable_altitude()
            alt_change = last_alt - current_alt  # 고도 변화량
            last_alt = current_alt
            
            print(f"[드론] 착륙 중... 고도: {current_alt:.2f}m, 하강률: {alt_change*20:.2f}m/s [{source}]", end='\r')
            
            # 지면 감지 (고도 0.1m 이하 + 변화 없음)
            if current_alt < 0.1 and abs(alt_change) < 0.01:  # 10cm 이하, 1cm 미만 변화
                ground_detect_count += 1
                if ground_detect_count > 20:  # 20회 연속 (1초간) 안정
                    break
            else:
                ground_detect_count = 0  # 리셋
            
            time.sleep(0.05)  # 20Hz (50ms)
        
        print(f"\n[드론] ✅ 착륙 완료: {current_alt:.2f}m")
    
    def get_status(self):
        """현재 상태 반환"""
        alt, alt_source = self.get_reliable_altitude()
        return {
            'connected': self.is_connected,
            'armed': self.is_armed,
            'altitude': alt,
            'altitude_source': alt_source,
            'vfr_alt': self.vfr_altitude,
            'ned_alt': self.ned_altitude,
            'rel_alt': self.relative_altitude,
            'gps': (self.current_lat, self.current_lon),
            'heading': self.current_heading,
            'position': (self.position_x, self.position_y, self.position_z),
            'emergency': self.emergency_hover_active,
            'command': self.current_command
        }
    
    def start_control(self):
        """제어 루프 시작 (20Hz)"""
        self.control_active = True
        
        def control_loop():
            while self.control_active:
                if self.emergency_hover_active:
                    # 긴급 호버링 중에는 계속 속도 0 전송
                    self._send_velocity_command(0, 0, 0)  # 모든 속도 0
                    time.sleep(0.05)  # 20Hz (50ms)
                    continue
                
                v, h1, h2, r = self.current_command
                
                # 속도 명령 지속 전송 (20Hz)
                if v != 0 or h1 != 0 or h2 != 0:
                    self._send_velocity_command(v, h1, h2)
                
                time.sleep(0.05)  # 20Hz (50ms = 0.05초)
        
        self.control_thread = threading.Thread(target=control_loop, daemon=True)
        self.control_thread.start()


# 테스트 코드
if __name__ == "__main__":
    drone = DroneCommandController('/dev/ttyACM0', 115200)
    
    # 연결
    if not drone.connect():
        exit()
    
    # 상태 확인
    time.sleep(2)  # 2초 대기
    status = drone.get_status()
    print(f"\n초기 상태: {status}")
    
    # 테스트 시나리오
    try:
        # 이륙 (FC 자동)
        drone.takeoff()  # FC 기본 고도, 자동 상승률
        time.sleep(3)
        
        # 회전 테스트 (최단 경로)
        print("\n=== 회전 테스트 ===")
        drone.rotate_to(90)   # 90도로 회전
        time.sleep(3)
        drone.rotate_to(270)  # 270도로 회전 (최단 경로는 반시계)
        time.sleep(3)
        drone.rotate_to(0)    # 북쪽으로 회전
        time.sleep(3)
        
        # 이동하면서 회전
        drone.set_command(0, 1.0, 0, 180)  # 전진하면서 180도 회전
        time.sleep(3)
        
        # 호버링
        drone.set_command(0, 0, 0)  # 모든 속도 0
        time.sleep(2)
        
        # 착륙
        drone.land()
        
        # 시동 해제
        drone.disarm()
        
    except KeyboardInterrupt:
        print("\n[중단] 안전 착륙...")
        drone.emergency_hover("사용자 중단")
        drone.land()
        drone.disarm()