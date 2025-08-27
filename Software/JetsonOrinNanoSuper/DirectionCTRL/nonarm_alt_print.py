
"""
altitude_monitor.py
실시간 고도 모니터링 (시동 없이)
"""

from pymavlink import mavutil
import time
import sys

# FC 연결
print("FC 연결 중...")
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
master.wait_heartbeat()
print("✅ FC 연결 성공\n")

# 데이터 스트림 요청
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # 10Hz
    1    # 활성화
)
time.sleep(1)

print("=== 실시간 고도 모니터링 === (Ctrl+C로 종료)\n")

try:
    while True:
        # 화면 클리어 (간단한 방법)
        print("\033[H\033[J", end="")  # 터미널 화면 클리어
        print("=== 실시간 고도 모니터링 === (Ctrl+C로 종료)\n")
        
        # GPS_RAW_INT
        msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            gps_msl = msg.alt / 1000
            sats = msg.satellites_visible
            fix = msg.fix_type
            print(f"GPS_RAW_INT:")
            print(f"  해발고도(MSL): {gps_msl:.2f}m")
            print(f"  위성: {sats}개, Fix: {fix}")
        else:
            print("GPS_RAW_INT: 수신 대기중...")
        
        print("")
        
        # GLOBAL_POSITION_INT
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            global_msl = msg.alt / 1000
            relative = msg.relative_alt / 1000
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f"GLOBAL_POSITION_INT:")
            print(f"  해발고도(MSL): {global_msl:.2f}m")
            print(f"  상대고도: {relative:.2f}m")
            print(f"  위치: {lat:.7f}, {lon:.7f}")
        else:
            print("GLOBAL_POSITION_INT: 수신 대기중...")
        
        print("")
        
        # VFR_HUD
        msg = master.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            baro = msg.alt
            climb = msg.climb
            print(f"VFR_HUD:")
            print(f"  기압고도: {baro:.2f}m")
            print(f"  상승률: {climb:.2f}m/s")
        else:
            print("VFR_HUD: 수신 대기중...")
        
        print("")
        
        # LOCAL_POSITION_NED
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            ned_z = -msg.z  # 음수가 위
            vz = -msg.vz
            print(f"LOCAL_POSITION_NED:")
            print(f"  NED고도: {ned_z:.2f}m")
            print(f"  수직속도: {vz:.2f}m/s")
        else:
            print("LOCAL_POSITION_NED: 수신 대기중...")
        
        print("")
        
        # ALTITUDE (지원하는 경우)
        msg = master.recv_match(type='ALTITUDE', blocking=False)
        if msg:
            amsl = msg.altitude_amsl
            local = msg.altitude_local
            rel = msg.altitude_relative
            print(f"ALTITUDE:")
            print(f"  AMSL: {amsl:.2f}m")
            print(f"  Local: {local:.2f}m")
            print(f"  Relative: {rel:.2f}m")
        else:
            print("ALTITUDE: 지원 안됨 또는 수신 대기중...")
        
        print("")
        print(f"업데이트: {time.strftime('%H:%M:%S')}")
        
        time.sleep(0.5)  # 0.5초마다 업데이트

except KeyboardInterrupt:
    print("\n\n종료합니다...")
    master.close()
    sys.exit(0)
```

실행하면 모든 고도 데이터가 실시간으로 업데이트됩니다. Ctrl+C로 종료 가능합니다.