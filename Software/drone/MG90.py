#MG90.py
import Jetson.GPIO as GPIO
import time

class MG90:
    def __init__(self, pin):
        """서보모터 제어 객체를 초기화합니다."""
        self.servo_pin = pin
        
        # GPIO 초기화
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # PWM 객체 생성 (50Hz 주파수)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        print(f"✅ GPIO {self.servo_pin}번 핀에서 서보모터 제어를 시작합니다.")

    def set_angle(self, angle):
        """지정한 각도(0-180)로 서보모터를 이동시킵니다."""
        # 각도를 듀티 사이클로 변환
        duty = (angle / 18) + 2.5
        
        # PWM 신호 출력
        GPIO.output(self.servo_pin, True)
        self.pwm.ChangeDutyCycle(duty)
        print(f"각도 설정: {angle}도 (듀티 사이클: {duty:.1f}%)")
        
        # 모터가 움직일 시간을 줌
        time.sleep(1)
        
        # 모터 떨림(jitter) 방지를 위해 신호 차단
        GPIO.output(self.servo_pin, False)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        """PWM을 정지하고 GPIO 리소스를 안전하게 해제합니다."""
        self.pwm.stop()
        GPIO.cleanup()
        print("🛑 서보모터 제어를 종료하고 GPIO를 정리했습니다.")

# --- 메인 실행 로직 ---
if __name__ == "__main__":
    # 서보모터를 33번 핀에 연결했다고 가정하고 객체 생성
    servo = MG90(pin=33)
    
    print("서보모터를 0도, 90도, 180도로 반복해서 움직입니다. (Ctrl+C로 종료)")

    try:
        while True:
            servo.set_angle(0)
            time.sleep(0.5)
            
            servo.set_angle(90)
            time.sleep(0.5)
            
            servo.set_angle(180)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    
    finally:
        # 프로그램 종료 시 반드시 cleanup 메소드를 호출하여 리소스를 해제
        servo.cleanup()
