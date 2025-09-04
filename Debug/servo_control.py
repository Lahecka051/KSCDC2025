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
        self.angle = 90  # 현재 각도 저장
        
        # GPIO 설정
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # PWM 설정 (50Hz)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
        # 초기 위치 설정 (90도 - idle)
        self.set_angle(90)
        print("[서보] 초기화 완료 (90도 - IDLE 위치)")
    
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
            print("[서보] 0도 - 투하 위치")  # 투하
        elif angle == 90:
            print("[서보] 90도 - IDLE 위치")  # idle
        elif angle == 180:
            print("[서보] 180도 - 재장전 위치")  # 재장전
        else:
            print(f"[서보] {angle}도로 회전")
    
    def drop(self):
        """소화볼 투하 (0도)"""
        print("[서보] 소화볼 투하 시작")
        self.set_angle(0)  # 투하
        
    def reload(self):
        """재장전 위치 (180도)"""
        print("[서보] 재장전 위치로 이동")
        self.set_angle(180)  # 재장전
        
    def idle(self):
        """대기 위치 (90도)"""
        print("[서보] IDLE 위치로 복귀")
        self.set_angle(90)  # idle
    
    def cleanup(self):
        """GPIO 정리"""
        try:
            self.pwm.stop()
            GPIO.cleanup()
            print("[서보] GPIO 정리 완료")
        except RuntimeError as e:
            # 수정: GPIO가 이미 정리된 경우 무시
            if "Not running on a RPi" not in str(e):
                print(f"[서보] GPIO 정리 중 오류 (무시 가능): {e}")
    
    def __del__(self):
        """소멸자 - 객체 삭제 시 GPIO 정리"""
        try:
            self.cleanup()
        except:
            pass