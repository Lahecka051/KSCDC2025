# servo_control.py
import Jetson.GPIO as GPIO
import time

class Servo:
    def __init__(self, servo_pin=33):
        """
        서보 모터 제어 클래스
        :param servo_pin: GPIO 핀 번호 (기본값: 33)
        """
        self.servo_pin = servo_pin
        self.angle = 0

        # GPIO 설정
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # PWM 설정 (50Hz)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)

        # 초기 위치 설정 (0도 - 재장전)
        self.set_angle(0)
        print("[서보] 초기화 완료 (0도 - 재장전 위치)")

    def set_angle(self, angle):
        """
        서보를 지정된 각도로 회전
        :param angle: 목표 각도 (0~180도)

        [최적화] PWM 신호 정지 추가 (서보 지터 방지)
        기존: duty cycle을 설정한 후 계속 유지 → 서보가 미세하게 떨림(지터)
              이는 PWM 신호의 타이밍 지터가 서보 위치 오차로 전달되기 때문
        수정: 목표 각도 도달 후 duty=0으로 설정하여 PWM 신호 정지
              서보는 마지막 위치를 기계적으로 유지하므로 안전함
        """
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180

        # duty cycle 계산 (2.5 ~ 12.5)
        duty = angle / 18 + 2.5
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        # [최적화] PWM 신호 정지 — 서보 지터 방지
        self.pwm.ChangeDutyCycle(0)
        self.angle = angle

        # 각도별 상태 출력
        if angle == 0:
            print("[서보] 0도 - 재장전(idle)")
        elif angle == 180:
            print("[서보] 180도 - 투하")
        else:
            print(f"[서보] {angle}도로 회전")

    def drop(self):
        """소화볼 투하 (180도)"""
        print("[서보] 소화볼 투하")
        self.set_angle(180)

    def reload(self):
        """재장전 위치 (0도)"""
        print("[서보] 재장전")
        self.set_angle(0)

    def cleanup(self):
        """GPIO 정리"""
        try:
            self.pwm.stop()
            GPIO.cleanup()
            print("[서보] GPIO 정리 완료")
        except RuntimeError as e:
            if "Not running on a RPi" not in str(e):
                print(f"[서보] GPIO 정리 중 오류 (무시 가능): {e}")

    def __del__(self):
        """소멸자 - 객체 삭제 시 GPIO 정리"""
        try:
            self.cleanup()
        except Exception:
            pass
