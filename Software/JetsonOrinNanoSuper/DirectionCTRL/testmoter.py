import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def run():
    drone = System()
    await drone.connect("serial:///dev/ttyTHS1:115200")
    
    # 연결 대기
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    
    print("드론 연결 완료")
    
    # 시동 걸기
    await drone.action.arm()
    print("시동 걸기 완료")
    
    # Offboard 모드로 80% 쓰로틀 5초간 유지
    try:
        # Offboard 모드 시작
        print("Offboard 모드 시작")
        
        # 80% 쓰로틀에 해당하는 상승 속도 설정 (약 4m/s)
        # VelocityBodyYawspeed(forward, right, down, yawspeed)
        # down이 음수이면 상승
        velocity = VelocityBodyYawspeed(0.0, 0.0, -4.0, 0.0)
        
        # 초기 setpoint 전송
        await drone.offboard.set_velocity_body(velocity)
        
        # Offboard 모드 시작
        await drone.offboard.start()
        
        # 5초간 80% 쓰로틀 유지
        print("80% 쓰로틀로 5초간 상승")
        await asyncio.sleep(5)
        
        # 호버링으로 전환 (쓰로틀 중단)
        hover_velocity = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        await drone.offboard.set_velocity_body(hover_velocity)
        print("호버링 모드")
        
        # 잠시 호버링
        await asyncio.sleep(1)
        
        # Offboard 모드 종료
        await drone.offboard.stop()
        print("Offboard 모드 종료")
        
    except Exception as e:
        print(f"Offboard 제어 중 오류: {e}")
        try:
            await drone.offboard.stop()
        except:
            pass
    
    # 시동 끄기
    await drone.action.disarm()
    print("시동 끄기 완료")

# 실행
asyncio.run(run())
