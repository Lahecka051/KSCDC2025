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
        self.angle = 0  # 수정: 초기값 0도로 변경
        
        # GPIO 설정
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # PWM 설정 (50Hz)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
        # 수정: 초기 위치 설정 (0도 - 재장전)
        self.set_angle(0)
        print("[서보] 초기화 완료 (0도 - 재장전 위치)")
    
    def set_angle(self, angle):
        """
        서보를 지정된 각도로 회전
        :param angle: 목표 각도 (0~180도)
        """
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
            
        # duty cycle 계산 (2.5 ~ 12.5)
        duty = angle / 18 + 2.5
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.angle = angle
        
        # 각도별 상태 출력
        if angle == 0:
            print("[서보] 0도 - 재장전(idle)")
        elif angle == 180:
            print("[서보] 180도 - 투하")
        else:
            print(f"[서보] {angle}도로 회전")
    
    def drop(self):
        """소화볼 투하 (180도)"""  # 수정: 180도가 투하
        print("[서보] 소화볼 투하")
        self.set_angle(180)
        
    def reload(self):
        """재장전 위치 (0도)"""  # 수정: 0도가 재장전
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
        except:
            pass