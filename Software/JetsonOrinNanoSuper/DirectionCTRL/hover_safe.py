
"""
reliable_hover.py
신뢰성 높은 고도 기반 비행 (강제 시동 해제 없음)
"""

from pymavlink import mavutil
import time

# FC 연결
print("FC 연결 중...")
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
master.wait_heartbeat()
print("✅ FC 연결 성공")

# 최대 샘플링 주파수로 데이터 스트림 요청
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    50,  # 50Hz (최대 샘플링)
    1    # 활성화
)
time.sleep(1)

def get_reliable_altitude():
    """가장 신뢰할 수 있는 고도 반환 (VFR_HUD 우선)"""
    # 1순위: VFR_HUD (기압계)
    msg = master.recv_match(type='VFR_HUD', blocking=False)
    if msg:
        return msg.alt, 'VFR_HUD'
    
    # 2순위: LOCAL_POSITION_NED (EKF 융합)
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        return -msg.z, 'LOCAL_NED'  # z는 음수가 위
    
    # 3순위: GLOBAL_POSITION_INT (상대고도)
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        return msg.relative_alt / 1000, 'GLOBAL_REL'
    
    return None, None

def wait_altitude_stable(target_alt, tolerance=0.3, timeout=30):
    """목표 고도 도달 대기"""
    start_time = time.time()
    stable_count = 0
    
    while time.time() - start_time < timeout:
        alt, source = get_reliable_altitude()
        if alt is not None:
            diff = abs(alt - target_alt)
            print(f"  고도: {alt:.2f}m (목표: {target_alt}m, 차이: {diff:.2f}m) [{source}]", end='\r')
            
            if diff < tolerance:
                stable_count += 1
                if stable_count > 5:  # 5회 연속 안정
                    print(f"\n  ✓ 목표 고도 도달: {alt:.2f}m")
                    return True
            else:
                stable_count = 0
        
        time.sleep(0.05)  # 20Hz 체크
    
    return False

# GUIDED 모드 설정
print("\nGUIDED 모드 설정...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4  # GUIDED
)
time.sleep(1)

# 초기 고도 확인
initial_alt, source = get_reliable_altitude()
if initial_alt is not None:
    print(f"초기 고도: {initial_alt:.2f}m [{source}]")
else:
    print("⚠️ 고도 데이터 수신 실패")
    exit()

# 시동 걸기
print("\n시동 걸기...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    1,  # param1: 1=시동
    0,  # param2: 0=정상 시동
    0, 0, 0, 0, 0
)
time.sleep(3)

# 1.5m 이륙
print("\n1.5m 이륙 시작...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,  # confirmation
    0,  # param1: 피치
    0,  # param2: 빈값
    0,  # param3: 상승률
    0,  # param4: yaw
    0,  # param5: 위도
    0,  # param6: 경도
    1.5 # param7: 목표 고도
)

# 이륙 완료 대기
if wait_altitude_stable(1.5, tolerance=0.2, timeout=10):
    print("✓ 이륙 완료!")
else:
    print("⚠️ 이륙 타임아웃")

# 15초 호버링 (높은 주파수로 위치 유지 명령)
print("\n15초 호버링 시작...")
hover_start = time.time()
cmd_count = 0

while time.time() - hover_start < 15:
    # 20Hz로 호버링 명령 전송
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # 속도만 사용
        0, 0, 0,  # 위치 (무시)
        0, 0, 0,  # 속도 (0 = 호버)
        0, 0, 0,  # 가속도 (무시)
        0, 0      # yaw (무시)
    )
    cmd_count += 1
    
    # 고도 모니터링
    if cmd_count % 20 == 0:  # 1초마다 출력
        alt, source = get_reliable_altitude()
        if alt is not None:
            remaining = 15 - (time.time() - hover_start)
            print(f"  호버링... {remaining:.1f}초 | 고도: {alt:.2f}m [{source}]")
    
    time.sleep(0.05)  # 20Hz

print("✓ 호버링 완료!")

# 부드러운 하강
print("\n착륙 모드 전환...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    9  # LAND
)

# 착륙 모니터링 (고주파수 샘플링)
print("착륙 중...")
landing_start = time.time()
last_alt = 999
ground_detect_count = 0

while True:
    alt, source = get_reliable_altitude()
    
    if alt is not None:
        # 고도 변화 감지
        alt_change = last_alt - alt
        last_alt = alt
        
        print(f"  착륙 중... 고도: {alt:.2f}m, 하강률: {alt_change*20:.2f}m/s [{source}]", end='\r')
        
        # 지면 감지 (고도 0.1m 이하 + 변화 없음)
        if alt < 0.1 and abs(alt_change) < 0.01:
            ground_detect_count += 1
            if ground_detect_count > 20:  # 1초간 안정
                print(f"\n✓ 착륙 완료! 최종 고도: {alt:.2f}m")
                break
        else:
            ground_detect_count = 0
    
    # 타임아웃 (30초)
    if time.time() - landing_start > 30:
        print("\n⚠️ 착륙 타임아웃")
        break
    
    time.sleep(0.05)  # 20Hz 샘플링

# 시동 끄기 (정상 모드)
print("\n시동 끄기...")
time.sleep(2)  # 안전 대기

# 여러 번 시도 (정상 시동 해제)
for i in range(3):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        i,  # confirmation (재시도 카운터)
        0,  # param1: 0=시동해제
        0,  # param2: 0=정상 시동해제 (강제 아님)
        0, 0, 0, 0, 0
    )
    time.sleep(1)
    
    # 시동 상태 확인
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("✓ 시동 해제 성공!")
        break
else:
    print("⚠️ 시동 해제 실패 (수동으로 해제 필요)")

print("\n✅ 미션 완료!")
master.close()
```

특징:
- **VFR_HUD** 기압계 고도 우선 사용 (가장 안정적)
- **50Hz** 데이터 스트림, **20Hz** 제어 명령
- **정상 시동 해제** (강제 사용 안함)
- 고도 안정성 체크로 안전한 전환