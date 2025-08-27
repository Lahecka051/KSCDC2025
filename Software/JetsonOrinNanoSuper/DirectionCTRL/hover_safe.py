
"""
standalone_hover.py
단독 실행 - 기압계 고도 사용 (H743V2) - 상세 주석 버전
"""

from pymavlink import mavutil
import time

# FC 연결
print("FC 연결 중...")
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
master.wait_heartbeat()
print("✅ FC 연결 성공")

# 데이터 스트림 요청
master.mav.request_data_stream_send(
    master.target_system,      # 타겟 시스템 ID (FC)
    master.target_component,   # 타겟 컴포넌트 ID
    mavutil.mavlink.MAV_DATA_STREAM_ALL,  # 요청할 스트림 타입 (모든 데이터)
    10,  # 전송률 (Hz)
    1    # 1=활성화, 0=비활성화
)
time.sleep(1)

# 초기 상태 확인
msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=2)
if msg:
    print(f"초기 기압 고도: {msg.alt:.2f}m")

# GUIDED 모드 설정
print("\nGUIDED 모드 설정...")
master.mav.set_mode_send(
    master.target_system,      # 타겟 시스템 ID
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # 모드 플래그 (커스텀 모드 사용)
    4  # 커스텀 모드 번호 (4 = GUIDED)
)
time.sleep(1)

# 시동 걸기
print("시동 걸기...")
master.mav.command_long_send(
    master.target_system,      # param1: 타겟 시스템 ID
    master.target_component,   # param2: 타겟 컴포넌트 ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # 명령 ID (시동/시동해제)
    0,  # confirmation (0 = 첫 번째 전송)
    1,  # param1: 1=시동, 0=시동해제
    0,  # param2: 0=정상모드, 21196=강제 시동해제
    0,  # param3: 예약됨
    0,  # param4: 예약됨
    0,  # param5: 예약됨
    0,  # param6: 예약됨
    0   # param7: 예약됨
)
time.sleep(3)

# 1.5m 이륙
print("1.5m 이륙...")
master.mav.command_long_send(
    master.target_system,      # 타겟 시스템 ID
    master.target_component,   # 타겟 컴포넌트 ID
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # 명령 ID (이륙)
    0,  # confirmation (0 = 첫 번째 전송)
    0,  # param1: 피치 각도 (무시됨)
    0,  # param2: 빈 값
    0,  # param3: 상승률 (0=자동)
    0,  # param4: Yaw 각도 (NaN=현재 방향 유지)
    0,  # param5: 위도 (0=현재 위치)
    0,  # param6: 경도 (0=현재 위치)
    1.5 # param7: 목표 고도 (미터)
)

# 이륙 대기
for i in range(5):
    msg = master.recv_match(type='VFR_HUD', blocking=False)
    if msg:
        print(f"이륙 중... 기압 고도: {msg.alt:.2f}m")
    
    # LOCAL_POSITION_NED도 확인
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        print(f"  NED 고도: {-msg.z:.2f}m")  # z는 음수가 위
    
    time.sleep(1)

print("이륙 완료")

# 15초 호버링
print("\n15초 호버링 시작...")
for i in range(15):
    # 속도 0으로 위치 유지
    master.mav.set_position_target_local_ned_send(
        0,                      # time_boot_ms (0 = 무시)
        master.target_system,   # 타겟 시스템 ID
        master.target_component,# 타겟 컴포넌트 ID
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # 좌표계 (바디 오프셋 NED)
        0b0000111111000111,     # type_mask (비트마스크: 사용하지 않을 필드 설정)
                                # bit 0: x 위치 무시
                                # bit 1: y 위치 무시
                                # bit 2: z 위치 무시
                                # bit 3: x 속도 사용 (0)
                                # bit 4: y 속도 사용 (0)
                                # bit 5: z 속도 사용 (0)
                                # bit 6: x 가속도 무시
                                # bit 7: y 가속도 무시
                                # bit 8: z 가속도 무시
                                # bit 9: 힘 무시
                                # bit 10: yaw 무시
                                # bit 11: yaw rate 무시
        0, 0, 0,  # x, y, z 위치 (미터) - 무시됨
        0, 0, 0,  # vx, vy, vz 속도 (m/s) - 0 = 현재 위치 유지
        0, 0, 0,  # ax, ay, az 가속도 (m/s^2) - 무시됨
        0,        # yaw (라디안) - 무시됨
        0         # yaw_rate (rad/s) - 무시됨
    )
    
    # 기압계 고도 + GPS 위치
    vfr_msg = master.recv_match(type='VFR_HUD', blocking=False)
    gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
    
    if vfr_msg and gps_msg:
        lat = gps_msg.lat / 1e7
        lon = gps_msg.lon / 1e7
        baro_alt = vfr_msg.alt
        print(f"호버링... {15-i}초 | GPS: {lat:.7f}, {lon:.7f} | 기압고도: {baro_alt:.2f}m")
    elif vfr_msg:
        print(f"호버링... {15-i}초 | 기압고도: {vfr_msg.alt:.2f}m")
    else:
        print(f"호버링... {15-i}초")
    
    time.sleep(1)

print("호버링 완료")

# 착륙
print("\n착륙 중...")
master.mav.set_mode_send(
    master.target_system,      # 타겟 시스템 ID
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # 모드 플래그
    9  # 커스텀 모드 번호 (9 = LAND)
)

# 착륙 모니터링 (기압계 고도 사용)
landing_count = 0
while landing_count < 30:  # 최대 30초
    msg = master.recv_match(type='VFR_HUD', blocking=False)
    if msg:
        baro_alt = msg.alt
        print(f"착륙 중... 기압고도: {baro_alt:.2f}m")
        
        # 0.2m 이하면 착륙 완료로 판단
        if baro_alt < 0.2:
            print("착륙 감지!")
            time.sleep(2)  # 안전 대기
            break
    
    landing_count += 1
    time.sleep(1)

print("착륙 완료")

# 시동 끄기
print("\n시동 끄기...")
master.mav.command_long_send(
    master.target_system,      # 타겟 시스템 ID
    master.target_component,   # 타겟 컴포넌트 ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # 명령 ID (시동/시동해제)
    0,      # confirmation (0 = 첫 번째 전송)
    0,      # param1: 0=시동해제, 1=시동
    21196,  # param2: 21196=강제 시동해제, 0=정상 시동해제
    0,      # param3: 예약됨
    0,      # param4: 예약됨
    0,      # param5: 예약됨
    0,      # param6: 예약됨
    0       # param7: 예약됨
)

print("✅ 미션 완료!")
master.close()
```

주요 파라미터 설명:
- **type_mask (비트마스크)**: 1로 설정된 비트는 해당 필드 무시, 0은 사용
- **21196**: 강제 시동 해제 매직 넘버
- **confirmation**: 재전송 시 증가 (0=첫 전송, 1=재전송...)