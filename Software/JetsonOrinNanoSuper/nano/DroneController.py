import time
from pymavlink import mavutil

class DroneController:
    """
    MicoAir743 V2와 Jetson Orin Nano를 위한 드론 제어 클래스.
    MAVLink를 통해 드론에 명령을 전송합니다.
    """
    def __init__(self, port='/dev/ttyTHS0', baud=57600):
        """
        DroneController 객체를 초기화하고 MAVLink 연결을 설정합니다.
        
        :param port: Jetson Orin Nano의 시리얼 포트 (기본값: '/dev/ttyTHS0')
        :param baud: 통신 보드레이트 (기본값: 57600)
        """
        self.master = mavutil.mavlink_connection(port, baud=baud)
        print("연결 대기 중...")
        self.master.wait_heartbeat()
        print("✅ 하트비트 수신 완료! 드론이 연결되었습니다.")

    def arm(self):
        """기체를 Arm(시동)합니다."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        print("🚁 기체 ARM")

    def disarm(self):
        """기체를 Disarm(시동 끄기)합니다."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        print("🛑 기체 DISARM")

    def takeoff(self, altitude):
        """지정한 고도로 이륙합니다. (단위: 미터)"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude)
        print(f"🛫 {altitude}m 고도로 이륙합니다.")

    def land(self):
        """현재 위치에 착륙합니다."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0)
        print("🛬 착륙 중...")

    def set_velocity(self, vx, vy, vz):
        """
        지정된 속도로 기체를 계속 움직입니다. 새로운 명령이 있을 때까지 유지됩니다.
        :param vx: 전/후 속도 (m/s, +는 전진)
        :param vy: 좌/우 속도 (m/s, +는 우진)
        :param vz: 상/하 속도 (m/s, +는 하강)
        """
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # 비트마스크: 위치와 가속도는 무시하고 속도만 제어
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        print(f"속도 설정: [전후: {vx} m/s, 좌우: {vy} m/s, 상하: {vz} m/s]")

    def hover(self):
        """모든 방향의 속도를 0으로 만들어 제자리에서 호버링합니다."""
        self.set_velocity(0, 0, 0)
        print("🚁 호버링")

    def goto_gps_location(self, lat, lon, alt):
        """
        지정된 GPS 좌표로 이동합니다.
        :param lat: 위도 (e.g., 35.1234567)
        :param lon: 경도 (e.g., 129.1234567)
        :param alt: 고도 (m, 지표면 기준)
        """
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # 비트마스크: 속도와 가속도는 무시하고 위치만 제어
            int(lat * 1e7),   # 위도를 정수형으로 변환
            int(lon * 1e7),   # 경도를 정수형으로 변환
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0)
        print(f"🗺️ GPS 좌표로 이동: [위도: {lat}, 경도: {lon}, 고도: {alt}m]")

# --- 이 파일이 직접 실행될 때만 아래 코드가 동작 (테스트 및 예제용) ---
if __name__ == '__main__':
    try:
        # 1. 드론 컨트롤러 객체 생성 및 연결
        drone = DroneController(port='/dev/ttyTHS0', baud=57600)

        # 2. Arm 및 이륙
        drone.arm()
        time.sleep(2)  # Arm 명령이 전달될 시간 확보
        drone.takeoff(5)
        print("이륙 완료까지 7초 대기...")
        time.sleep(7)

        # 3. 전진 (3초간 유지)
        print("\n--- 속도 제어 테스트 ---")
        drone.set_velocity(1.5, 0, 0) # 초속 1.5m로 전진 시작
        time.sleep(3)

        # 4. 우진 (3초간 유지)
        drone.set_velocity(0, 1.5, 0) # 초속 1.5m로 우진 시작
        time.sleep(3)

        # 5. 호버링 (3초간 유지)
        drone.hover()
        time.sleep(3)

        # 6. GPS 좌표 이동 (가상의 부산역 근처 좌표)
        print("\n--- GPS 이동 테스트 ---")
        busan_station_lat = 35.1151
        busan_station_lon = 129.0423
        drone.goto_gps_location(busan_station_lat, busan_station_lon, 10)
        print("목표 지점 도착까지 15초 대기...")
        time.sleep(15)

        # 7. 착륙 및 Disarm
        print("\n--- 비행 종료 ---")
        drone.land()
        time.sleep(10) # 착륙 완료까지 대기
        drone.disarm()

    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        print("프로그램 종료.")
