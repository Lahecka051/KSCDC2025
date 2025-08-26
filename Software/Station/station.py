# station_program.py

import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x
from camera_module import Docking # Docking 클래스 불러오기

# --- 1. 하드웨어 설정 ---
GPIO.setmode(GPIO.BCM)

SERVO_FB_PIN = 17
SERVO_LR_PIN = 18
SERVO_AMMO_PIN = 27

XSHUT_PINS = [22, 23, 24, 25]

PWM_FREQUENCY = 50

GPIO.setup(SERVO_FB_PIN, GPIO.OUT)
GPIO.setup(SERVO_LR_PIN, GPIO.OUT)
GPIO.setup(SERVO_AMMO_PIN, GPIO.OUT)

pwm_fb = GPIO.PWM(SERVO_FB_PIN, PWM_FREQUENCY)
pwm_lr = GPIO.PWM(SERVO_LR_PIN, PWM_FREQUENCY)
pwm_ammo = GPIO.PWM(SERVO_AMMO_PIN, PWM_FREQUENCY)

pwm_fb.start(0)
pwm_lr.start(0)
pwm_ammo.start(0)

# --- 2. 변수 및 상수 ---
TOLERANCE_MM_TOF = 5
TARGET_DISTANCE_MM_TOF = 100

TOLERANCE_MM_CAMERA = 10 # 카메라로 이동할 때의 허용 오차
TARGET_DISTANCE_MM_CAMERA = 500 # 카메라로 도달할 목표 거리 (예시)

SERVO_360_STOP = 7.5
SERVO_360_FORWARD = 8.5
SERVO_360_BACKWARD = 6.5
SERVO_360_LEFT = 8.5
SERVO_360_RIGHT = 6.5

AMMO_LOAD_SPEED = 8.5

# --- 3. 함수 정의 ---
def setup_tof_sensors(i2c_bus, xshut_pins):
    for pin in xshut_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    time.sleep(0.1)
    
    sensors = []
    
    for i, xshut_pin in enumerate(xshut_pins):
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

def get_sensor_data(sensors):
    distances = []
    try:
        for tof in sensors:
            distances.append(tof.range)
        return distances
    except Exception as e:
        print(f"센서 읽기 오류: {e}")
        return [None] * len(sensors)

def calculate_offset_tof(distances):
    if len(distances) < 4:
        return 0, 0, False
        
    dist1, dist2, dist3, dist4 = distances
    
    offset_front_back = dist1 - dist3
    offset_left_right = dist2 - dist4
    
    is_centered = (abs(dist1 - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF and
                   abs(dist2 - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF and
                   abs(dist3 - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF and
                   abs(dist4 - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF)
    
    return offset_front_back, offset_left_right, is_centered

def control_servos_tof(offset_fb, offset_lr):
    # 앞뒤 서보 제어
    if offset_fb > TOLERANCE_MM_TOF:
        pwm_fb.ChangeDutyCycle(SERVO_360_BACKWARD)
        print("  -> ToF: 뒤로 이동")
    elif offset_fb < -TOLERANCE_MM_TOF:
        pwm_fb.ChangeDutyCycle(SERVO_360_FORWARD)
        print("  -> ToF: 앞으로 이동")
    else:
        pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> ToF: 앞뒤 정렬 완료")

    # 좌우 서보 제어
    if offset_lr > TOLERANCE_MM_TOF:
        pwm_lr.ChangeDutyCycle(SERVO_360_LEFT)
        print("  -> ToF: 왼쪽으로 이동")
    elif offset_lr < -TOLERANCE_MM_TOF:
        pwm_lr.ChangeDutyCycle(SERVO_360_RIGHT)
        print("  -> ToF: 오른쪽으로 이동")
    else:
        pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> ToF: 좌우 정렬 완료")
    
    time.sleep(0.1)

def control_servos_camera(cmd):
    if cmd == "left":
        pwm_lr.ChangeDutyCycle(SERVO_360_LEFT)
        print("  -> 카메라: 왼쪽으로 이동")
    elif cmd == "right":
        pwm_lr.ChangeDutyCycle(SERVO_360_RIGHT)
        print("  -> 카메라: 오른쪽으로 이동")
    elif "left" in cmd:
        pwm_lr.ChangeDutyCycle(SERVO_360_LEFT)
        print("  -> 카메라: 왼쪽으로 이동")
    elif "right" in cmd:
        pwm_lr.ChangeDutyCycle(SERVO_360_RIGHT)
        print("  -> 카메라: 오른쪽으로 이동")
    else:
        pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> 카메라: 좌우 정렬 완료")

def load_ammo(count):
    print(f"\n🚀 {count}발의 소화탄을 장전합니다.")
    for i in range(count):
        print(f"  -> 장전 중... {i+1} / {count}")
        pwm_ammo.ChangeDutyCycle(AMMO_LOAD_SPEED)
        time.sleep(1.0)
        pwm_ammo.ChangeDutyCycle(SERVO_360_STOP)
        time.sleep(0.5)
    print("✅ 장전 완료!")

# --- 4. 메인 루프 ---
if __name__ == '__main__':
    try:
        # 카메라 모듈 인스턴스 생성
        print("📷 카메라를 사용하여 시각적 정렬을 시작합니다.")
        docking = Docking()

        # --- 1단계: 카메라를 이용한 좌우 정렬 ---
        camera_aligned_lateral = False
        while not camera_aligned_lateral:
            cmd = docking.get_command()
            
            if cmd is None:
                print("마커를 찾을 수 없습니다. 드론을 수동으로 이동시키세요.")
            elif cmd == "stop":
                print("✅ 카메라 좌우 정렬 완료. 다음 단계로 넘어갑니다.")
                camera_aligned_lateral = True
            else:
                control_servos_camera(cmd)
            time.sleep(0.5)

        # --- 2단계: 카메라로 거리 측정 및 앞뒤 이동 ---
        print("\n📏 카메라로 거리를 측정하여 앞뒤로 이동합니다.")
        pwm_lr.ChangeDutyCycle(SERVO_360_STOP) # 좌우 서보 정지
        distance_aligned = False
        while not distance_aligned:
            distance = docking.calculate_distance()
            
            if distance is None:
                print("마커를 찾을 수 없습니다. 수동으로 거리를 맞춰주세요.")
                continue

            print(f"   -> 카메라로 측정한 거리: {distance:.2f}mm")

            if abs(distance - TARGET_DISTANCE_MM_CAMERA) <= TOLERANCE_MM_CAMERA:
                print("✅ 카메라 거리 정렬 완료. 다음 단계로 넘어갑니다.")
                distance_aligned = True
            elif distance > TARGET_DISTANCE_MM_CAMERA:
                pwm_fb.ChangeDutyCycle(SERVO_360_BACKWARD)
                print("   -> 너무 가까움, 뒤로 이동")
            else:
                pwm_fb.ChangeDutyCycle(SERVO_360_FORWARD)
                print("   -> 너무 멈, 앞으로 이동")
            
            time.sleep(0.5)

        # 카메라 자원 해제
        docking.cleanup()
        pwm_fb.ChangeDutyCycle(SERVO_360_STOP) # 앞뒤 서보 정지

        # --- 3단계: ToF 센서를 이용한 정밀 매칭 및 장전 ---
        print("\n⚙️ ToF 센서로 정밀 매칭을 시작합니다.")
        i2c = busio.I2C(board.SCL, board.SDA)
        tof_sensors = setup_tof_sensors(i2c, XSHUT_PINS)
        
        if len(tof_sensors) < 4:
            print("🚨 4개의 ToF 센서가 모두 연결되지 않았습니다. 프로그램을 종료합니다.")
        else:
            ammo_count = 1
            while True:
                distances = get_sensor_data(tof_sensors)
                
                if None in distances:
                    print("⚠️ 센서 데이터를 읽을 수 없습니다. 다시 시도합니다.")
                    time.sleep(1)
                    continue
                
                print(f"\n🔍 센서 거리 (mm): 12시={distances[0]}, 3시={distances[1]}, 6시={distances[2]}, 9시={distances[3]}")
                
                offset_fb, offset_lr, is_centered = calculate_offset_tof(distances)
                
                if is_centered:
                    print("\n✅ 드론의 소화탄 장착부가 완벽하게 매칭되었습니다.")
                    pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
                    pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
                    load_ammo(ammo_count)
                    break 
                else:
                    print(f"   -> 정렬 필요: 앞뒤 오차={offset_fb:.2f}mm, 좌우 오차={offset_lr:.2f}mm")
                    control_servos_tof(offset_fb, offset_lr)
                
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n👋 프로그램을 종료합니다.")
    finally:
        pwm_fb.stop()
        pwm_lr.stop()
        pwm_ammo.stop()
        GPIO.cleanup()
