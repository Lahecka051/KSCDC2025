import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x
from Doc_match import Docking

# 통신 클라이언트 라이브러리 임포트
from rpi_client import RPiClient
import queue

class DroneStation:
    """
    드론 스테이션의 도킹 및 장전 과정을 제어하는 클래스.
    카메라, ToF 센서, 서보 모터, PC와의 통신을 통합 관리합니다.
    """

    def __init__(self, pc_host):
        # --- 1. 하드웨어 설정 ---
        GPIO.setmode(GPIO.BCM)
        self.SERVO_FB_PIN = 17      # 전, 후진 서보
        self.SERVO_LR_PIN = 18      # 좌, 우 서보
        self.SERVO_AMMO_PIN = 27    # 발 수 구분 서보

        self.XSHUT_PINS = [22, 23, 24, 25]  # ToF 핀번호(4개)
        self.PWM_FREQUENCY = 50     # PWM 신호 주파수(50Hz)

        GPIO.setup(self.SERVO_FB_PIN, GPIO.OUT)
        GPIO.setup(self.SERVO_LR_PIN, GPIO.OUT)
        GPIO.setup(self.SERVO_AMMO_PIN, GPIO.OUT)

        self.pwm_fb = GPIO.PWM(self.SERVO_FB_PIN, self.PWM_FREQUENCY)
        self.pwm_lr = GPIO.PWM(self.SERVO_LR_PIN, self.PWM_FREQUENCY)
        self.pwm_ammo = GPIO.PWM(self.SERVO_AMMO_PIN, self.PWM_FREQUENCY)
        
        self.pwm_fb.start(0)
        self.pwm_lr.start(0)
        self.pwm_ammo.start(0)

        # --- 2. 변수 및 상수 ---
        self.TOLERANCE_MM_TOF = 5
        self.TARGET_DISTANCE_MM_TOF = 100
        self.TOLERANCE_PX_CAMERA = 5
        self.TARGET_Y_RELATIVE_PX = -100

        self.SERVO_360_STOP = 7.5
        self.SERVO_360_FORWARD = 8.5
        self.SERVO_360_BACKWARD = 6.5
        self.SERVO_360_LEFT = 8.5
        self.SERVO_360_RIGHT = 6.5
        self.AMMO_LOAD_SPEED = 8.5

        # --- 3. 통신 클라이언트 초기화 ---
        self.pc_host = pc_host
        self.client = RPiClient(host=self.pc_host)

    def setup_tof_sensors(self):
        """ToF 센서 순차 활성화 및 I2C 주소 재할당"""
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        for pin in self.XSHUT_PINS:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        time.sleep(0.1)
        
        sensors = []
        for i, xshut_pin in enumerate(self.XSHUT_PINS):
            GPIO.output(xshut_pin, GPIO.HIGH)
            time.sleep(0.1)
            
            try:
                tof = adafruit_vl53l0x.VL53L0X(i2c_bus)
                new_address = 0x29 + i
                tof.set_address(new_address)
                sensors.append(adafruit_vl53l0x.VL53L0X(i2c_bus, address=new_address))
                print(f"✅ 센서 {i+1}에 주소 0x{new_address:x} 할당 완료.")
            except ValueError:
                print(f"⚠️ 센서 {i+1}를 찾을 수 없거나 주소 설정에 실패했습니다. 연결을 확인하세요.")
        return sensors

    def get_sensor_data(self, sensors):
        """모든 ToF 센서에서 현재 거리 측정"""
        distances = []
        try:
            for tof in sensors:
                distances.append(tof.range)
            return distances
        except Exception as e:
            print(f"센서 읽기 오류: {e}")
            return [None] * len(sensors)

    def control_servos_tof(self, offset_fb, offset_lr):
        """ToF 센서 측정 오차 기반 서보 제어"""
        if abs(offset_fb) > self.TOLERANCE_MM_TOF:
            self.pwm_fb.ChangeDutyCycle(self.SERVO_360_BACKWARD if offset_fb > 0 else self.SERVO_360_FORWARD)
            print(f"  -> ToF: {'뒤로' if offset_fb > 0 else '앞으로'} 이동")
        else:
            self.pwm_fb.ChangeDutyCycle(self.SERVO_360_STOP)
            print("  -> ToF: 앞뒤 정렬 완료")
            
        if abs(offset_lr) > self.TOLERANCE_MM_TOF:
            self.pwm_lr.ChangeDutyCycle(self.SERVO_360_LEFT if offset_lr > 0 else self.SERVO_360_RIGHT)
            print(f"  -> ToF: {'왼쪽' if offset_lr > 0 else '오른쪽'}으로 이동")
        else:
            self.pwm_lr.ChangeDutyCycle(self.SERVO_360_STOP)
            print("  -> ToF: 좌우 정렬 완료")
        time.sleep(0.1)

    def load_ammo(self, count):
        """장전 서보 구동"""
        print(f"\n🚀 {count}발의 소화탄을 장전합니다.")
        for i in range(count):
            print(f"  -> 장전 중... {i+1} / {count}")
            self.pwm_ammo.ChangeDutyCycle(self.AMMO_LOAD_SPEED)
            time.sleep(1.0)
            self.pwm_ammo.ChangeDutyCycle(self.SERVO_360_STOP)
            time.sleep(0.5)
        print("✅ 장전 완료!")

    def run_station_process(self, ammo_count):
        """카메라와 ToF 센서를 사용하여 도킹 및 장전 과정을 실행하는 함수"""
        try:
            docking = Docking()

            # --- 1단계: 카메라를 이용한 좌우 정렬 및 전진 ---
            print("\n📷 카메라를 사용하여 시각적 정렬을 시작합니다.")
            
            while True:
                dot_x_relative, dot_y_relative, found = docking.get_coordinates()
                
                if not found:
                    print("⚠️ 빨간 마커를 찾을 수 없습니다. 전진을 계속합니다.")
                    self.pwm_fb.ChangeDutyCycle(self.SERVO_360_FORWARD)
                    self.pwm_lr.ChangeDutyCycle(self.SERVO_360_STOP)
                    time.sleep(0.5)
                    continue

                if abs(dot_x_relative) > self.TOLERANCE_PX_CAMERA:
                    self.pwm_lr.ChangeDutyCycle(self.SERVO_360_LEFT if dot_x_relative > 0 else self.SERVO_360_RIGHT)
                    print(f"  -> 카메라: {'왼쪽' if dot_x_relative > 0 else '오른쪽'}으로 이동")
                else:
                    self.pwm_lr.ChangeDutyCycle(self.SERVO_360_STOP)
                    print("
