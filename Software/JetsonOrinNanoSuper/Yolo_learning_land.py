import cv2
import numpy as np
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from ultralytics import YOLO

# YOLO 모델 로드
model = YOLO("h_marker.pt")  # H 마커 인식용 모델

# PID 제어 클래스
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt=0.05):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# 드론 연결
async def connect_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("드론 연결 대기 중...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ 드론 연결 완료!")
            break
    return drone

# 드론 착륙 로직
async def landing_logic(drone):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다.")
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_center = (w // 2, h // 2)

    pid_x = PIDController(kp=0.002, ki=0, kd=0.001)
    pid_y = PIDController(kp=0.002, ki=0, kd=0.001)

    # 오프보드 모드 시작
    print("🛫 Offboard 제어 시작 시도")
    try:
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0, 1)
        await drone.offboard.start()
        print("✅ Offboard 시작됨")
    except OffboardError as e:
        print(f"❌ Offboard 시작 실패: {e._result.result}")
        return

    print("🚀 YOLO 기반 H 마커 자동 착륙 시작")

    landing_ready_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 카메라 프레임을 읽을 수 없습니다.")
            break

        results = model(frame, verbose=False)
        detected = False
        h_area = 0
        h_center = None

        for r in results:
            if r.boxes is not None and len(r.boxes) > 0:
                for box in r.boxes:
                    cls = int(box.cls.item())
                    if cls == 0:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        h_center = (cx, cy)
                        h_area = (x2 - x1) * (y2 - y1)
                        detected = True
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(frame, h_center, 5, (0, 0, 255), -1)
                        break

        if detected:
            dx = h_center[0] - frame_center[0]
            dy = h_center[1] - frame_center[1]

            vx = max(min(pid_x.compute(-dx), 0.5), -0.5)
            vy = max(min(pid_y.compute(-dy), 0.5), -0.5)

            print(f"🎯 H 감지: center={h_center}, dx={dx}, dy={dy}, area={h_area}")

            if abs(dx) < 20 and abs(dy) < 20 and h_area > 12000:
                landing_ready_count += 1
                print(f"📍 착륙 준비 상태 유지 {landing_ready_count}/10")
                if landing_ready_count >= 20:
                    print("🛬 착륙 지점 도달 → 착륙 실행")
                    await drone.action.land()
                    break
            else:
                landing_ready_count = 0

            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(vx, vy, 0.0, 0.0)
            )
        else:
            print("🔍 H 마커 미검출")
            landing_ready_count = 0

        cv2.imshow("Landing Camera (YOLO)", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

        await asyncio.sleep(0.05)  # 루프 안정화

    cap.release()
    cv2.destroyAllWindows()

    # 착륙 완료 확인
    print("🛬 착륙 중 → 착륙 완료 대기...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("✅ 드론 착륙 완료")
            break

# 메인 실행 루틴
async def main():
    drone = await connect_drone()

    print("🔐 Arm 시도 중...")
    await drone.action.arm()
    print("✅ 드론 Arm 완료")

    await landing_logic(drone)

if __name__ == "__main__":
    asyncio.run(main())
