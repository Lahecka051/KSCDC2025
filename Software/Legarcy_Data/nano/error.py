"""
standalone_hover.py
단독 실행 - 시동 → GUIDED → 1.5m 상승 → 15초 호버링 → 착륙 (GPS 출력)
"""

from pymavlink import mavutil
import time

# FC 연결
print("FC 연결 중...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)  # 보드레이트 확인
master.wait_heartbeat()
print("✅ FC 연결 성공")

# 데이터 스트림 요청
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    10,  # 10Hz
    1
)
time.sleep(1)

# 초기 GPS 상태 확인
msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
if msg:
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000
    sats = msg.satellites_visible
    print(f"초기 GPS: {lat:.7f}, {lon:.7f}, {alt:.1f}m, 위성:{sats}개")

# GUIDED 모드 설정
print("\nGUIDED 모드 설정...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4  # GUIDED
)
time.sleep(1)

# 시동 걸기
print("시동 걸기...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(3)

# 5m 이륙
print("5m 이륙...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0,
    5  # 목표 고도
)

# 이륙 대기
for i in range(5):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000
        print(f"이륙 중... GPS: {lat:.7f}, {lon:.7f}, 고도: {alt:.2f}m")
    time.sleep(1)

print("이륙 완료")

# 15초 호버링 (속도 0 명령)
print("\n15초 호버링 시작...")
for i in range(15):
    # 속도 0으로 위치 유지
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,  # 위치
        0, 0, 0,  # 속도 (0 = 호버링)
        0, 0, 0,  # 가속도
        0, 0  # yaw
    )
    
    # GPS 데이터 수신
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000
        print(f"호버링... {15-i}초 | GPS: {lat:.7f}, {lon:.7f}, 고도: {alt:.2f}m")
    else:
        # GPS_RAW_INT도 시도
        msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f"호버링... {15-i}초 | GPS: {lat:.7f}, {lon:.7f}")
        else:
            print(f"호버링... {15-i}초")
    
    time.sleep(1)

print("호버링 완료")

# 착륙
print("\n착륙 중...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    9  # LAND
)

# 착륙 중 GPS 모니터링
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000
        print(f"착륙 중... GPS: {lat:.7f}, {lon:.7f}, 고도: {alt:.2f}m")
        if alt < 0.1:  # 10cm 미만이면 착륙 완료
            break
    time.sleep(0.5)

print("착륙 완료")

# 시동 끄기
print("\n시동 끄기...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 21196, 0, 0, 0, 0, 0
)

# 최종 GPS
msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
if msg:
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000
    print(f"최종 GPS: {lat:.7f}, {lon:.7f}, {alt:.1f}m")

print("✅ 미션 완료!")
master.close()
