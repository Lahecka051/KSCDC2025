from pymavlink import mavutil
import time
import math

class DroneController:
    def __init__(self, connection_string='/dev/ttyACM0', baudrate=115200):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.is_armed = False
        
        # 고도 정보
        self.set_alt = 2.0        # 설정 고도 (이륙, 주행 중 유지할 고도)
        self.archived_alt = None  # 이륙 완료 판정 고도 (set_alt의 95%)


        # GPS 정보
        self.home_lat = None  # 홈 위치 추가
        self.home_lon = None  # 홈 위치 추가
        
    def connect(self):
        """FC 연결"""
        try:
            print("[드론] FC 연결 중...")
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            self.master.wait_heartbeat()
            print("[드론] FC 연결 성공")
            
            # 데이터 스트림 요청
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                50,
                1
            )
            
            # 홈 GPS 좌표 저장
            print("[드론] 홈 GPS 좌표 읽는 중...")
            for i in range(10):  # 최대 10번 시도
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    self.home_lat = msg.lat / 1e7
                    self.home_lon = msg.lon / 1e7
                    print(f"[드론] 홈 위치 저장: 위도 {self.home_lat:.7f}, 경도 {self.home_lon:.7f}")
                    return True
            
            print("[드론] 홈 GPS 좌표 읽기 실패")
            return False
            
        except Exception as e:
            print(f"[드론] 연결 실패: {e}")
            return False
        
    def set_mode_guided(self):
        """GUIDED 모드 설정"""
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode
        )
        time.sleep(0.1)
             
    def set_mode_rtl(self):
        """RTL(Return To Launch) 모드 설정"""
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6  # RTL mode
        )
        time.sleep(0.1)
        
    def set_mode_land(self):
        """LAND 모드 설정"""
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9  # LAND mode
        )
        time.sleep(0.1)
        
    def set_mode_brake(self):
        """BRAKE 모드 설정"""
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            17  # BRAKE mode
        )
        time.sleep(0.1)    
        
    def arm(self):
        """시동"""
        
        # GUIDED 모드 설정
        self.set_mode_guided()
        
        # ARM 명령
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)
        print("\n[드론] 시동 중...")
        
        # 시동 확인
        for i in range(10):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED               
                if self.is_armed:
                    print("[드론] 시동 성공")
                    return True
            time.sleep(0.1)
        print("[드론] 시동 실패")
        return False
        
    def disarm(self):
        """시동 끄기"""
        print("\n[드론] 시동 끄기...")
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)
    
    def takeoff(self, altitude=None):
        if altitude:
            self.set_alt = altitude
        self.archived_alt = self.set_alt * 0.95  # 이륙 완료 판정 고도 (set_alt의 95%)
        """이륙 - 목표 고도 도달 시 완료 판정"""
        print(f"\n[드론] 이륙 명령 (목표: {self.set_alt}m)...")

        # TAKEOFF 명령
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, self.set_alt
        )
    
        # 이륙 완료 판정
        timeout = 20  # 최대 20초 대기
        start_time = time.time()
    
        while time.time() - start_time < timeout:
             # VFR_HUD에서 상대 고도 읽기
            msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=0.02)
            if msg:
                current_alt = msg.alt  # 상대 고도
            
                # 또는 GLOBAL_POSITION_INT 사용 시
                # msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
                # if msg:
                #     current_alt = msg.relative_alt / 1000.0  # mm를 m로 변환

                print(f"  현재 고도: {current_alt:.2f}m / 목표: {self.set_alt}m", end='\r')
            
                # 목표 고도의 95% 도달 시 완료 판정
                if current_alt >= self.archived_alt:
                    print(f"\n[드론] 이륙 완료 (고도: {current_alt:.2f}m)")
                    return True
        
        print(f"\n[드론] 이륙 시간 초과 ({timeout}초)")
        return False
        
    def set_command(self, cmd):

        # cmd가 list이고 길이가 4인지 확인
        if isinstance(cmd, list) and len(cmd) == 4:
            vx = cmd[0]  # 전진/후진 (North)
            vy = -cmd[1]  # 좌/우 (East, 좌측이 +이므로 부호 반전)
            vz = -cmd[2]  # 상승/하강 (Down, 상승이 +이므로 부호 반전)
            yaw_deg = cmd[3]
            if yaw_deg > 180:
                yaw_deg = yaw_deg - 360
            rotation = math.radians(yaw_deg)  # -π ~ π 라디안으로 변환
        else:
            raise ValueError("Command must be a list with 4 elements")

        # MAVLink 명령 전송
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED 프레임
            0b0000101111000111,  # type_mask: 속도와 yaw 사용
            0, 0, 0,  # 위치 (사용 안 함)
            vx, vy, vz,  # 속도 (NED)
            0, 0, 0,  # 가속도 (사용 안 함)
            rotation, 0  # yaw, yaw_rate
        )
    
    def read_altitude(self):
        """현재 상대 고도를 읽어서 반환"""
        msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        if msg:
            altitude = msg.alt  # 상대 고도 (미터)
            print(f"[드론] 현재 고도: {altitude:.2f}m")
            return altitude
        else:
            print("[드론] 고도 읽기 실패")
            return None
    
        
    def read_gps(self):
        """현재 GPS 위치를 딕셔너리로 반환"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            gps_data = {
                "lat": lat,
                "lon": lon
            }
            # 2번째 줄에 출력 (ANSI 이스케이프 코드 사용)
            print(f"\r\033[K[드론] GPS: 위도 {lat:.7f}, 경도 {lon:.7f}", end='', flush=True)
            return gps_data
        else:
            print("\r\033[K[드론] GPS 읽기 실패", end='', flush=True)
            return None

    def goto_gps(self, latitude, longitude, altitude=None):
        if altitude is None:
            altitude = self.set_alt
        # 커서를 위로 올리고 첫 번째 줄에 출력
        print(f"\033[A\r\033[K[GPS 이동] 목표: {latitude:.7f}, {longitude:.7f}, 고도: {altitude:.1f}m\n", end='', flush=True)

        # GPS 좌표로 이동 명령
        self.master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # 상대 고도 사용
            0b0000111111111000,  # type_mask: 위치만 사용
            int(latitude * 1e7),  # 위도 (1e7 스케일)
            int(longitude * 1e7),  # 경도 (1e7 스케일)
            altitude,  # 고도 (미터)
            0, 0, 0,  # 속도 (사용 안 함)
            0, 0, 0,  # 가속도 (사용 안 함)
            0, 0  # yaw, yaw_rate (사용 안 함)
        )
    
    def goto_home(self):
        """홈 위치로 복귀"""
        if self.home_lat is None or self.home_lon is None:
            print("[드론] 홈 위치가 설정되지 않음")
            return False
        
        print(f"[드론] 홈으로 복귀 중...")
        while True:
            self.goto_gps(self.home_lat, self.home_lon, self.set_alt)
            
            # GPS 읽기 및 None 체크 추가
            current_gps = self.read_gps()
            if not current_gps:
                time.sleep(0.1)
                continue
                
            distance = self.get_distance_metres(
                self.home_lat, self.home_lon,
                current_gps.get("lat"), current_gps.get("lon")
            )
            if distance <= 2:
                print("[드론] 홈 위치 도착")
                break
            time.sleep(0.1)
        return True

    def get_distance_metres(self,lat1, lon1, lat2, lon2):
        """두 GPS 좌표 사이의 거리를 미터 단위로 계산하는 함수 (하버사인 공식)"""
        R = 6371e3  # 지구의 반지름 (미터)
        
        # 위도, 경도를 라디안으로 변환
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance
        
if __name__ == "__main__":
    
    drone = DroneController()
    
    try:
        if not drone.connect():  # 연결 실패 체크
            exit(1)
            
        if not drone.arm():  # 시동 실패 체크
            exit(1)
            
        if not drone.takeoff(2):  # 이륙 실패 시 착륙
            drone.set_mode_land()
            exit(1)
            
        drone.set_command([1, 0, 0, 90]) # 임의의 명령
        
    except KeyboardInterrupt:
        print("\n[긴급] Ctrl+C 감지 - 착륙 모드 전환")
        if drone.is_armed:  # 시동 상태 확인
            drone.set_mode_land()
            time.sleep(15)  # 착륙 대기
            drone.disarm()
        print("[종료] 프로그램 종료")
        
    except Exception as e:
        print(f"[오류] {e}")
        if drone.is_armed:
            drone.set_mode_land()

    
    
    
