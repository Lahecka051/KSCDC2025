# station_program.py

import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x
from camera_module import Docking

# --- 1. 하드웨어 설정 ---
GPIO.setmode(GPIO.BCM)

SERVO_FB_PIN = 17    # 전, 후진 서보 지정
SERVO_LR_PIN = 18    # 좌, 우 서보 지정
SERVO_AMMO_PIN = 27  # 발 수 구분 서보 지정

XSHUT_PINS = [22, 23, 24, 25]  #ToF 핀번호(현 4개)

PWM_FREQUENCY = 50  #PWM 신호 주파수(50Hz)

GPIO.setup(SERVO_FB_PIN, GPIO.OUT)
GPIO.setup(SERVO_LR_PIN, GPIO.OUT)
GPIO.setup(SERVO_AMMO_PIN, GPIO.OUT)  # ~ 출력 모드 설정

pwm_fb = GPIO.PWM(SERVO_FB_PIN, PWM_FREQUENCY)
pwm_lr = GPIO.PWM(SERVO_LR_PIN, PWM_FREQUENCY)
pwm_ammo = GPIO.PWM(SERVO_AMMO_PIN, PWM_FREQUENCY)  # 각 서보 핀 PWM 인스턴스 생성 및 주파수 설정

pwm_fb.start(0)
pwm_lr.start(0)
pwm_ammo.start(0)  #서보 초기 설정

# --- 2. 변수 및 상수 ---
TOLERANCE_MM_TOF = 5           # ToF 센서 허용 오차
TARGET_DISTANCE_MM_TOF = 100   # 4개 ToF 센서 목표 거리

# 카메라 기반 정렬 허용 오차
TOLERANCE_PX_CAMERA = 5
# ToF 센서 구동을 위한 카메라 기반 전진 목표 y축 위치 (조정 필요)
# 더 작은 음수 값일수록 드론이 y축에 더 가까이 있다는 의미
TARGET_Y_RELATIVE_PX = -100        # Dot y축 위치가 이 값 도달시 ToF 센서 정렬로 넘어감

SERVO_360_STOP = 7.5
SERVO_360_FORWARD = 8.5
SERVO_360_BACKWARD = 6.5
SERVO_360_LEFT = 8.5
SERVO_360_RIGHT = 6.5

AMMO_LOAD_SPEED = 8.5   # ~ DutyCycle 값

# 소화탄 장전 매커니즘을 위한 변수
# True면 2발 장전, False면 1발 장전 (Jetson과의 통신으로 결정)
is_first_docking = True 

# --- 3. 함수 정의 ---
def setup_tof_sensors(i2c_bus, xshut_pins):   # ToF 센서 순차활성, I2C 주소 재할당 -> 모두 사용
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

def get_sensor_data(sensors):    # 모든 ToF 센서에서 현재 거리 측정 -> 리스트 형태 반환
    distances = []
    try:
        for tof in sensors:
            distances.append(tof.range)
        return distances
    except Exception as e:
        print(f"센서 읽기 오류: {e}")
        return [None] * len(sensors)

def control_servos_tof(offset_fb, offset_lr):     # ToF 센서 측정 앞뒤좌우 거리 오차 값 기반 서보 제어
    if abs(offset_fb) > TOLERANCE_MM_TOF:
        if offset_fb > 0:
            pwm_fb.ChangeDutyCycle(SERVO_360_BACKWARD)
            print("  -> ToF: 뒤로 이동")
        else:
            pwm_fb.ChangeDutyCycle(SERVO_360_FORWARD)
            print("  -> ToF: 앞으로 이동")
    else:
        pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> ToF: 앞뒤 정렬 완료")
        
    if abs(offset_lr) > TOLERANCE_MM_TOF:
        if offset_lr > 0:
            pwm_lr.ChangeDutyCycle(SERVO_360_LEFT)
            print("  -> ToF: 왼쪽으로 이동")
        else:
            pwm_lr.ChangeDutyCycle(SERVO_360_RIGHT)
            print("  -> ToF: 오른쪽으로 이동")
    else:
        pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> ToF: 좌우 정렬 완료")
    time.sleep(0.1)

def load_ammo(count):     # 인자로 받은 횟수만큼 장전 서보 구동 -> 소화탄 장전
    print(f"\n🚀 {count}발의 소화탄을 장전합니다.")
    for i in range(count):
        print(f"  -> 장전 중... {i+1} / {count}")
        pwm_ammo.ChangeDutyCycle(AMMO_LOAD_SPEED)
        time.sleep(1.0)
        pwm_ammo.ChangeDutyCycle(SERVO_360_STOP)
        time.sleep(0.5)
    print("✅ 장전 완료!")

# --- 4. 메인 루프 ---
if __name__ == '__main__':
    try:
        print("--- 드론 스테이션 프로그램 시작 ---")
        
        # 카메라 모듈 인스턴스 생성
        docking = Docking()

        # --- 1단계: 카메라를 이용한 좌우 정렬 및 전진 ---
        print("\n📷 카메라를 사용하여 시각적 정렬을 시작합니다.")
        
        # 전진 중 좌우 정렬을 함께 수행
        while True:
            dot_x_relative, dot_y_relative, found = docking.get_coordinates()
            
            if not found:
                print("⚠️ 빨간 마커를 찾을 수 없습니다. 전진을 계속합니다.")
                pwm_fb.ChangeDutyCycle(SERVO_360_FORWARD)
                pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
                time.sleep(0.5)
                continue

            # 좌우 서보 제어
            if abs(dot_x_relative) > TOLERANCE_PX_CAMERA:
                if dot_x_relative > 0:
                    pwm_lr.ChangeDutyCycle(SERVO_360_LEFT)
                    print("  -> 카메라: 왼쪽으로 이동")
                else:
                    pwm_lr.ChangeDutyCycle(SERVO_360_RIGHT)
                    print("  -> 카메라: 오른쪽으로 이동")
            else:
                pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
                print("  -> 카메라: 좌우 정렬 완료")

            # 앞뒤 서보 제어 (ToF 센서 구동을 위한 전진)
            if dot_y_relative > TARGET_Y_RELATIVE_PX:
                pwm_fb.ChangeDutyCycle(SERVO_360_FORWARD)
                print("  -> 카메라: 앞으로 이동")
            else:
                print("✅ 카메라 정렬 및 거리 확보 완료. ToF 센서 단계로 넘어갑니다.")
                pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
                pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
                break
            time.sleep(0.1)

        docking.cleanup()

        # --- 2단계: ToF 센서를 이용한 정밀 매칭 및 장전 ---
        print("\n⚙️ ToF 센서로 정밀 매칭을 시작합니다.")
        i2c = busio.I2C(board.SCL, board.SDA)
        tof_sensors = setup_tof_sensors(i2c, XSHUT_PINS)
        
        if len(tof_sensors) < 4:
            print("🚨 4개의 ToF 센서가 모두 연결되지 않았습니다. 프로그램을 종료합니다.")
        else:
            # 소화탄 장전 발수 결정 @@@@@@@@@@@@@@@@@ 수정 필요 관제 센터 -> 스테이션 로그 전송 -> 발수 결정
            ammo_count = 2 if is_first_docking else 1
            print(f"-> 소화탄 장전 발수: {ammo_count} 발")
            
            while True:
                distances = get_sensor_data(tof_sensors)
                
                if None in distances:
                    print("⚠️ 센서 데이터를 읽을 수 없습니다. 다시 시도합니다.")
                    time.sleep(1)
                    continue
                
                # ToF 센서 데이터를 사용하여 매칭을 확인
                is_all_matched = (
                    abs(distances[0] - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF and
                    abs(distances[1] - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF and
                    abs(distances[2] - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF and
                    abs(distances[3] - TARGET_DISTANCE_MM_TOF) <= TOLERANCE_MM_TOF
                )
                
                if is_all_matched:
                    print("\n✅ 드론의 소화탄 장착부가 완벽하게 매칭되었습니다.")
                    pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
                    pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
                    load_ammo(ammo_count)
                    break
                else:
                    offset_fb = distances[0] - distances[2]
                    offset_lr = distances[1] - distances[3]
                    print(f"   -> 정렬 필요: 앞뒤 오차={offset_fb:.2f}mm, 좌우 오차={offset_lr:.2f}mm")
                    control_servos_tof(offset_fb, offset_lr)
                    
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n👋 프로그램을 종료합니다.")
    finally:
        pwm_fb.stop()
        pwm_lr.stop()
        pwm_ammo.stop()
        GPIO.cleanup()
