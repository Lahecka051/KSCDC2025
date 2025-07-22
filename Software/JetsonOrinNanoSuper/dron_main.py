import cv2
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from yolo_trt import YOLOTRT

# ========================
# 드론 제어 클래스
# ========================
class DroneController:
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.drone = System()
        self.loop.run_until_complete(self.connect_and_prepare())

    async def connect_and_prepare(self):
        await self.drone.connect(system_address="udp://:14540")
        print("[DRONE] Connected")
        print("[DRONE] Arming...")
        await self.drone.action.arm()
        print("[DRONE] Taking off to 10m...")
        await self.drone.action.takeoff()
        await asyncio.sleep(8)  # 이륙 안정화 대기
        print("[DRONE] Takeoff complete")

    def move_forward(self):
        print("[DRONE] Moving forward (2m)")
        self.loop.run_until_complete(self._move_velocity(2.0, 0.0, 0.0))

    def move_diag_right_up(self):
        print("[DRONE] Diagonal right up (2m)")
        self.loop.run_until_complete(self._move_velocity(1.5, 1.5, 0.0))

    def move_diag_left_down(self):
        print("[DRONE] Diagonal left down (2m)")
        self.loop.run_until_complete(self._move_velocity(-1.5, -1.5, 0.0))

    def patrol_path(self):
        print("[DRONE] Executing patrol mission")
        self.loop.run_until_complete(self._patrol_mission())

    def return_to_home(self):
        print("[DRONE] Returning home")
        self.loop.run_until_complete(self.drone.action.return_to_launch())

    async def _move_velocity(self, vx, vy, vz, duration=3):
        from mavsdk.offboard import VelocityNedYaw
        await self.drone.offboard.start()
        cmd = VelocityNedYaw(vx, vy, vz, 0.0)
        for _ in range(duration * 10):
            await self.drone.offboard.set_velocity_ned(cmd)
            await asyncio.sleep(0.1)
        await self.drone.offboard.stop()

    async def _patrol_mission(self):
        mission_items = [
            MissionItem(37.401, 126.900, 30, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 0, 1),
            MissionItem(37.401, 126.901, 30, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 0, 1),
            MissionItem(37.400, 126.901, 30, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 0, 1),
            MissionItem(37.400, 126.900, 30, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, 0, 1),
        ]
        mission_plan = MissionPlan(mission_items)
        await self.drone.mission.upload_mission(mission_plan)
        await self.drone.mission.start_mission()

# ========================
# GStreamer 파이프라인
# ========================
def gstreamer_pipeline(
        sensor_id=0,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink"
    )

# ========================
# 메인 로직
# ========================
def main():
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    fire_detector = YOLOTRT("/home/user/models/yolov8n-fire.engine")
    drone = DroneController()

    patrol_mode = True
    last_fire_detected = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        h, w, _ = frame.shape
        h_step = h // 2
        w_step = w // 3

        fire_detected = False
        fire_zone = -1
        zone_idx = 1

        # 6분할 영역 탐색
        for row in range(2):
            for col in range(3):
                x1, y1 = col * w_step, row * h_step
                x2, y2 = x1 + w_step, y1 + h_step
                roi = frame[y1:y2, x1:x2]

                detections = fire_detector.detect(roi)
                fire_flag = any(cls == 0 for cls, conf in detections)

                if fire_flag:
                    fire_detected = True
                    fire_zone = zone_idx
                    cv2.putText(frame, f"FIRE {zone_idx}", (x1+10, y1+30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                else:
                    cv2.putText(frame, f"{zone_idx}", (x1+10, y1+30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                zone_idx += 1

        # 드론 제어
        if fire_detected:
            patrol_mode = False
            if fire_zone == 2:
                drone.move_forward()
            elif fire_zone == 3:
                drone.move_diag_right_up()
            elif fire_zone == 4:
                drone.move_diag_left_down()
            last_fire_detected = True
        else:
            if not last_fire_detected:
                if patrol_mode:
                    drone.patrol_path()
            else:
                drone.return_to_home()
                last_fire_detected = False
                patrol_mode = True

        cv2.imshow("Fire Detection - 6 Zones", frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
