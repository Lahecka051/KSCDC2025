import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x

# --- 1. 하드웨어 설정 ---
# GPIO 모드 설정 (BCM 모드는 GPIO 번호를 사용)
GPIO.setmode(GPIO.BCM)

# 서보모터 GPIO 핀 번호 정의
# 앞뒤/좌우 서보모터는 360도 서보(연속 회전)입니다.
SERVO_FB_PIN = 17  # 앞뒤 서보
SERVO_LR_PIN = 18  # 좌우 서보
SERVO_AMMO_PIN = 27 # 소화탄 장전 서보

# ToF 센서의 XSHUT GPIO 핀 번호 정의 (4개 센서)
XSHUT_PINS = [22, 23, 24, 25]

# PWM 설정
PWM_FREQUENCY = 50 # 서보모터는 보통 50Hz를 사용

# PWM 객체 생성
GPIO.setup(SERVO_FB_PIN, GPIO.OUT)
GPIO.setup(SERVO_LR_PIN, GPIO.OUT)
GPIO.setup(SERVO_AMMO_PIN, GPIO.OUT)

pwm_fb = GPIO.PWM(SERVO_FB_PIN, PWM_FREQUENCY)
pwm_lr = GPIO.PWM(SERVO_LR_PIN, PWM_FREQUENCY)
pwm_ammo = GPIO.PWM(SERVO_AMMO_PIN, PWM_FREQUENCY)

# PWM 시작 (초기 듀티 사이클 0)
pwm_fb.start(0)
pwm_lr.start(0)
pwm_ammo.start(0)

# --- 2. 변수 및 상수 ---
# 허용 오차 (밀리미터)
TOLERANCE_MM = 5
# 목표 거리 (밀리미터)
TARGET_DISTANCE_MM = 100

# 360도 서보모터 회전 제어용 듀티 사이클
# 7.5: 정지, 7.5보다 작으면 한 방향, 7.5보다 크면 반대 방향
SERVO_360_STOP = 7.5
SERVO_360_FORWARD = 8.5   # 앞으로 이동 (듀티 사이클은 튜닝 필요)
SERVO_360_BACKWARD = 6.5  # 뒤로 이동
SERVO_360_LEFT = 8.5      # 왼쪽으로 이동
SERVO_360_RIGHT = 6.5     # 오른쪽으로 이동

# 소화탄 장전 360도 서보의 회전 속도 (튜닝 필요)
AMMO_LOAD_SPEED = 8.5

# --- 3. 함수 정의 ---
def setup_tof_sensors(i2c_bus, xshut_pins):
    """
    XSHUT 핀을 이용해 여러 ToF 센서의 I2C 주소를 설정합니다.
    """
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
    """
    4개의 ToF 센서로부터 거리를 읽어옵니다.
    """
    distances = []
    try:
        for tof in sensors:
            distances.append(tof.range)
        return distances
    except Exception as e:
        print(f"센서 읽기 오류: {e}")
        return [None] * len(sensors)

def calculate_offset(distances):
    """
    센서 거리를 기반으로 앞뒤 및 좌우 오차를 계산합니다.
    """
    if len(distances) < 4:
        return 0, 0, False
        
    dist1, dist2, dist3, dist4 = distances
    
    offset_front_back = dist1 - dist3
    offset_left_right = dist2 - dist4
    
    is_centered = (abs(dist1 - TARGET_DISTANCE_MM) <= TOLERANCE_MM and
                   abs(dist2 - TARGET_DISTANCE_MM) <= TOLERANCE_MM and
                   abs(dist3 - TARGET_DISTANCE_MM) <= TOLERANCE_MM and
                   abs(dist4 - TARGET_DISTANCE_MM) <= TOLERANCE_MM)
    
    return offset_front_back, offset_left_right, is_centered

def control_servos(offset_fb, offset_lr):
    """
    계산된 오차에 따라 앞뒤/좌우 360도 서보모터를 제어합니다.
    """
    # 앞뒤 서보 제어
    if offset_fb > TOLERANCE_MM:
        # 드론이 너무 앞에 있으므로 뒤로 이동
        pwm_fb.ChangeDutyCycle(SERVO_360_BACKWARD)
        print("  -> 뒤로 이동")
    elif offset_fb < -TOLERANCE_MM:
        # 드론이 너무 뒤에 있으므로 앞으로 이동
        pwm_fb.ChangeDutyCycle(SERVO_360_FORWARD)
        print("  -> 앞으로 이동")
    else:
        # 정렬 완료 시 정지
        pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> 앞뒤 정렬 완료")

    # 좌우 서보 제어
    if offset_lr > TOLERANCE_MM:
        # 드론이 너무 오른쪽에 있으므로 왼쪽으로 이동
        pwm_lr.ChangeDutyCycle(SERVO_360_LEFT)
        print("  -> 왼쪽으로 이동")
    elif offset_lr < -TOLERANCE_MM:
        # 드론이 너무 왼쪽에 있으므로 오른쪽으로 이동
        pwm_lr.ChangeDutyCycle(SERVO_360_RIGHT)
        print("  -> 오른쪽으로 이동")
    else:
        # 정렬 완료 시 정지
        pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
        print("  -> 좌우 정렬 완료")
    
    time.sleep(0.1) # 짧은 대기 시간

def load_ammo(count):
    """
    소화탄을 장전하는 서보모터를 회전시킵니다.
    """
    print(f"\n🚀 {count}발의 소화탄을 장전합니다.")
    for i in range(count):
        print(f"  -> 장전 중... {i+1} / {count}")
        # 360도 서보모터는 듀티 사이클로 속도 제어
        pwm_ammo.ChangeDutyCycle(AMMO_LOAD_SPEED)
        time.sleep(1.0) # 회전 시간 (조정 필요)
        
        # 정지 듀티 사이클로 회전 정지
        pwm_ammo.ChangeDutyCycle(SERVO_360_STOP)
        time.sleep(0.5)
    
    print("✅ 장전 완료!")

# --- 4. 메인 루프 ---
if __name__ == '__main__':
    try:
        # I2C 버스 초기화
        i2c = busio.I2C(board.SCL, board.SDA)

        # ToF 센서 초기화 및 주소 할당
        tof_sensors = setup_tof_sensors(i2c, XSHUT_PINS)
        
        if len(tof_sensors) < 4:
            print("🚨 4개의 ToF 센서가 모두 연결되지 않았습니다. 프로그램을 종료합니다.")
        else:
            # 소화탄 장전 수 설정 (1발 또는 2발)
            ammo_count = 1 
            
            while True:
                distances = get_sensor_data(tof_sensors)
                
                if None in distances:
                    print("⚠️ 센서 데이터를 읽을 수 없습니다. 다시 시도합니다.")
                    time.sleep(1)
                    continue
                
                print(f"\n🔍 센서 거리 (mm): 12시={distances[0]}, 3시={distances[1]}, 6시={distances[2]}, 9시={distances[3]}")
                
                offset_fb, offset_lr, is_centered = calculate_offset(distances)
                
                if is_centered:
                    print("\n✅ 드론의 소화탄 장착부가 완벽하게 매칭되었습니다.")
                    
                    # 모든 서보 정지
                    pwm_fb.ChangeDutyCycle(SERVO_360_STOP)
                    pwm_lr.ChangeDutyCycle(SERVO_360_STOP)
                    
                    load_ammo(ammo_count)
                    break 
                else:
                    print(f"   -> 정렬 필요: 앞뒤 오차={offset_fb:.2f}mm, 좌우 오차={offset_lr:.2f}mm")
                    control_servos(offset_fb, offset_lr)
                
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n👋 프로그램을 종료합니다.")
    finally:
        # 종료 시 GPIO 자원 정리
        pwm_fb.stop()
        pwm_lr.stop()
        pwm_ammo.stop()
        GPIO.cleanup()
