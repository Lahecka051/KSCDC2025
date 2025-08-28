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
        self.gps_altitude = 0.0      # GPS 해발고도 (절대고도)
        self.baro_altitude = 0.0     # 기압계 고도 (VFR_HUD)
        self.ekf_altitude = 0.0      # EKF 융합 고도 (LOCAL_POSITION_NED)
        self.relative_alt = 0.0      # 상대 고도 (이륙 지점 기준)
        self.set_alt = 5.0            # 설정 고도 (이륙 후 유지할 고도)

        # GPS 정보
        self.current_lat = None
        self.current_lon = None
        
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
                10,  # 10Hz로 데이터 요청
                1
            )
            
        except Exception as e:
            print(f"[드론] ❌ 연결 실패: {e}")
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
        
    def set_mode_break(self):
        """BREAK 모드 설정"""
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            17  # BREAK mode
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
        
        # 시동 확인
        for i in range(10):
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                self.is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED               
            if self.is_armed:
                print("[드론] ✅ 시동 성공")
                return True
            else:
                print("[드론] ❌ 시동 실패")
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
        # for i in range(10):
        #     if not self.is_armed:
        #         print("[드론] ✅ 시동 해제 성공")
        #         return True
        #     time.sleep(0.5)

        # print("[드론] ⚠️ 시동 해제 실패")
        # return False
        time.sleep(2)
    
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
        
    def set_command(self, cmd):

        # cmd가 list이고 길이가 4인지 확인
        if isinstance(cmd, list) and len(cmd) == 4:
            vx = cmd[0]  # 전진/후진 (North)
            vy = -cmd[1]  # 좌/우 (East, 좌측이 +이므로 부호 반전)
            vz = -cmd[2]  # 상승/하강 (Down, 상승이 +이므로 부호 반전)
            rotation = math.radians(int(cmd[3]))  # 드론 방향 (0~359도, 시계방향)
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
        
    def goto_gps(self, latitude, longitude, altitude=None):
        """
        GPS 좌표로 이동
        latitude: 위도
        longitude: 경도
        altitude: 고도
        """ 
        
        if altitude is None:
            altitude = self.set_alt  # 5M 고도 설정
            print(f"[GPS 이동] 위도: {latitude:.7f}, 경도: {longitude:.7f}, 고도: ({altitude:.1f}m)")
        
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

if __name__ == "__main__":
    drone = DroneController()
    drone.connect()
    drone.arm()
    drone.takeoff(2)
    drone.set_command([1, 0, 0, 90])

    
    
    
