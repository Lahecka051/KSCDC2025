from pymavlink import mavutil
import time
import math

# [최적화] 비행 모드 상수 정의
# 기존: 매직 넘버(4, 6, 9, 17)를 직접 사용하여 의미 파악 어려움
# 수정: 이름 있는 상수로 추출하여 가독성 및 유지보수성 향상
FLIGHT_MODE_GUIDED = 4
FLIGHT_MODE_RTL = 6
FLIGHT_MODE_LAND = 9
FLIGHT_MODE_BRAKE = 17

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
        self.home_lat = None
        self.home_lon = None

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
            for i in range(10):
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

    def _set_mode(self, mode_id):
        """[최적화] 비행 모드 전환 공통 메서드 추출
        기존: set_mode_guided/rtl/land/brake 각각에 동일한 MAVLink 전송 코드 중복
        수정: 공통 로직을 하나의 메서드로 통합하여 코드 중복 제거 (DRY 원칙)
        """
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(0.1)

    def set_mode_guided(self):
        """GUIDED 모드 설정"""
        self._set_mode(FLIGHT_MODE_GUIDED)

    def set_mode_rtl(self):
        """RTL(Return To Launch) 모드 설정"""
        self._set_mode(FLIGHT_MODE_RTL)

    def set_mode_land(self):
        """LAND 모드 설정"""
        self._set_mode(FLIGHT_MODE_LAND)

    def set_mode_brake(self):
        """BRAKE 모드 설정"""
        self._set_mode(FLIGHT_MODE_BRAKE)

    def arm(self):
        """시동"""

        # GUIDED 모드 설정
        self.set_mode_guided()

        # 시동 전 홈 위치 리셋 (중요!)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0, 1, 0, 0, 0, 0, 0, 0  # 현재 위치를 홈으로 설정
        )
        time.sleep(0.5)

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
        """이륙 - 목표 고도 도달 시 완료 판정

        [최적화] 독스트링 위치 수정
        기존: self.archived_alt 할당 이후에 독스트링이 위치 → Python이 독스트링으로 인식 못함
        수정: def 바로 아래로 독스트링 이동하여 정상적인 함수 문서화
        """
        if altitude:
            self.set_alt = altitude
        self.archived_alt = self.set_alt * 0.95

        print(f"\n[드론] 이륙 명령 (목표: {self.set_alt}m)...")

        # TAKEOFF 명령
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.set_alt
        )

        # 이륙 완료 판정
        timeout = 20
        start_time = time.time()

        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
            if msg:
                current_alt = msg.relative_alt / 1000.0

                print(f"  현재 고도: {current_alt:.2f}m / 목표: {self.set_alt}m", end='\r')

                if current_alt >= self.archived_alt:
                    print(f"\n[드론] 이륙 완료 (고도: {current_alt:.2f}m)")
                    return True

        print(f"\n[드론] 이륙 시간 초과 ({timeout}초)")
        return False

    def set_command(self, cmd):
        if not self.master:
            print("[드론] FC 연결이 없습니다")
            return False

        if isinstance(cmd, list) and len(cmd) == 4:
            vx = cmd[0]
            vy = -cmd[1]
            vz = -cmd[2]
            # [최적화] yaw 정규화 간소화
            # 기존: if yaw_deg > 180: yaw_deg = yaw_deg - 360 (0~360 범위만 처리)
            # 수정: 모듈로 연산으로 임의의 각도를 -180~180 범위로 정규화
            yaw_deg = ((cmd[3] + 180) % 360) - 180
            rotation = math.radians(yaw_deg)
        else:
            raise ValueError("Command must be a list with 4 elements")

        try:
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000101111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                rotation, 0
            )
            return True
        except Exception as e:
            print(f"[드론] 명령 전송 실패: {e}")
            return False

    def read_altitude(self):
        """현재 상대 고도를 읽어서 반환"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            altitude = msg.relative_alt / 1000.0
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
            print(f"\r\033[K[드론] GPS: 위도 {lat:.7f}, 경도 {lon:.7f}", end='', flush=True)
            return gps_data
        else:
            print("\r\033[K[드론] GPS 읽기 실패", end='', flush=True)
            return None

    def goto_gps(self, latitude, longitude, altitude=None):
        if altitude is None:
            altitude = self.set_alt
        print(f"\033[A\r\033[K[GPS 이동] 목표: {latitude:.7f}, {longitude:.7f}, 고도: {altitude:.1f}m\n", end='', flush=True)

        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        time.sleep(0.05)

    def goto_home(self):
        """홈 위치로 복귀

        [최적화] 무한 루프에 타임아웃 추가
        기존: while True 무한 루프 — GPS 오류 지속 시 영원히 빠져나오지 못함
        수정: 60초 타임아웃 추가, 초과 시 RTL 모드로 안전하게 전환
        """
        if self.home_lat is None or self.home_lon is None:
            print("[드론] 홈 위치가 설정되지 않음")
            return False

        print(f"[드론] 홈으로 복귀 중...")
        start_time = time.time()
        GOTO_HOME_TIMEOUT = 60  # 최대 60초

        while time.time() - start_time < GOTO_HOME_TIMEOUT:
            self.goto_gps(self.home_lat, self.home_lon, self.set_alt)

            current_gps = self.read_gps()
            if not current_gps:
                time.sleep(0.1)
                continue

            distance = self.get_distance_metres(
                current_gps.get("lat"), current_gps.get("lon"),
                self.home_lat, self.home_lon
            )

            if distance <= 2:
                print("[드론] 홈 위치 도착")
                return True

            if distance > 500:
                print("[드론] 위치 오류 감지 - RTL 모드 전환")
                self.set_mode_rtl()
                return False

            time.sleep(0.1)

        # [최적화] 타임아웃 시 RTL 모드로 안전하게 전환
        print(f"[드론] 홈 복귀 타임아웃 ({GOTO_HOME_TIMEOUT}초) - RTL 모드 전환")
        self.set_mode_rtl()
        return False

    def get_distance_metres(self, lat1, lon1, lat2, lon2):
        """두 GPS 좌표 사이의 거리를 미터 단위로 계산하는 함수 (하버사인 공식)

        [최적화] 매개변수 self, lat1 사이 공백 누락 수정 (PEP 8)
        """
        R = 6371e3

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
