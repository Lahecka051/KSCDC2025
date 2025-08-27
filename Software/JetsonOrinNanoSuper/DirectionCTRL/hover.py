"""
standalone_hover.py
단독 실행 - 시동 → GUIDED → 1.5m 상승 → 15초 호버링 → 착륙
"""

from pymavlink import mavutil
import time

# FC 연결
print("FC 연결 중...")
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=115200)
# master.wait_heartbeat()
print("✅ FC 연결 성공")

# GUIDED 모드 설정
print("GUIDED 모드 설정...")
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

# 2m 이륙
print("2m 이륙...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0,
    2 # 목표 고도
)

# 이륙 대기
time.sleep(5)
print("이륙 완료")

# 15초 호버링 (속도 0 명령)
print("15초 호버링 시작...")
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
print("\n호버링 완료")

# 착륙
print("착륙 중...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    9  # LAND
)

# 착륙 대기
time.sleep(10)

# 시동 끄기
print("시동 끄기...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 21196, 0, 0, 0, 0, 0
)

print("✅ 미션 완료!")
