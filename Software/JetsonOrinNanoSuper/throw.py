import Jetson.GPIO as GPIO
import time
import math

# ===================== 1. 환경 설정 =====================

g = 9.81  # 중력가속도 (m/s^2)

# 서보모터 설정
SERVO_PIN = 33       # Jetson 핀 번호 (BOARD 모드 기준) 번호 맞춰 수정 필요 @@
FREQ = 50            # 50Hz (20ms 주기)
MIN_US = 500         # 최소 펄스폭 (us)
MAX_US = 2500        # 최대 펄스폭 (us)
ANGLE_RANGE = 180    # 서보 최대 각도

# 드론 위치/목표 위치 (예시값)
drone_pos = (37.1234, 127.5678, 30.0)  # 위도, 경도, 고도
fire_pos = (37.1238, 127.5682, 0.0)    # 목표 위도, 경도, 지면 고도

# ===================== 2. 거리 계산 함수 =====================

def haversine_distance(lat1, lon1, lat2, lon2):
    """두 GPS 좌표 사이의 거리(m)"""
    R = 6371000  # 지구 반경(m)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ===================== 3. 투척 각도/속도 계산 =====================

def compute_throw_parameters(drone_pos, fire_pos, initial_angle_deg=45):
    """
    초기 각도를 입력받아 필요한 속도 계산
    """
    d = haversine_distance(drone_pos[0], drone_pos[1], fire_pos[0], fire_pos[1])
    dz = drone_pos[2] - fire_pos[2]
    theta = math.radians(initial_angle_deg)

    denominator = 2 * (math.cos(theta)**2) * (d * math.tan(theta) - dz)
    if denominator <= 0:
        raise ValueError("투척 각도가 너무 낮거나 목표 지점이 너무 가까움")

    v0 = math.sqrt((g * d**2) / denominator)
    return initial_angle_deg, v0, d, dz

# ===================== 4. 서보 제어 클래스 =====================

class Servo:
    def __init__(self, pin, freq=50, min_us=500, max_us=2500, angle=180):
        self.pin = pin
        self.freq = freq
        self.min_us = min_us
        self.max_us = max_us
        self.angle = angle

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)

    def servo_angle(self, degrees):
        """각도를 PWM duty로 변환해 서보 제어"""
        degrees = max(0, min(self.angle, degrees))
        total_range = self.max_us - self.min_us
        us = self.min_us + (total_range * degrees / self.angle)
        duty_cycle = (us / 20000) * 100  # 20ms 기준 duty 비율
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

# ===================== 5. 메인 동작 =====================

if __name__ == "__main__":
    try:
        # 1) 투척 각도/속도 계산
        theta, v0, d, dz = compute_throw_parameters(drone_pos, fire_pos, initial_angle_deg=45)
        print(f"[투척 계산] 거리: {d:.2f} m, 고도차: {dz:.2f} m")
        print(f"투척 각도: {theta:.2f}°, 초기 속도: {v0:.2f} m/s")

        # 2) 서보 모터 제어 (뚜껑 열기 → 유지 → 닫기)
        servo = Servo(SERVO_PIN, freq=FREQ, min_us=MIN_US, max_us=MAX_US, angle=ANGLE_RANGE)
        print("[서보 동작] 90도 열기")
        servo.servo_angle(90)
        time.sleep(1.5)
        print("[서보 동작] 0도 닫기")
        servo.servo_angle(0)
        time.sleep(0.5)

    except Exception as e:
        print("[오류]", e)
    finally:
        servo.cleanup()
