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
                    print("  -> 카메라: 좌우 정렬 완료")

                if dot_y_relative > self.TARGET_Y_RELATIVE_PX:
                    self.pwm_fb.ChangeDutyCycle(self.SERVO_360_FORWARD)
                    print("  -> 카메라: 앞으로 이동")
                else:
                    print("✅ 카메라 정렬 및 거리 확보 완료. ToF 센서 단계로 넘어갑니다.")
                    self.pwm_fb.ChangeDutyCycle(self.SERVO_360_STOP)
                    self.pwm_lr.ChangeDutyCycle(self.SERVO_360_STOP)
                    break
                time.sleep(0.1)

            docking.cleanup()

            # --- 2단계: ToF 센서를 이용한 정밀 매칭 및 장전 ---
            print("\n⚙️ ToF 센서로 정밀 매칭을 시작합니다.")
            tof_sensors = self.setup_tof_sensors()
            
            if len(tof_sensors) < 4:
                print("🚨 4개의 ToF 센서가 모두 연결되지 않았습니다. 장전을 시작할 수 없습니다.")
                return False
            else:
                print(f"-> 소화탄 장전 발수: {ammo_count} 발")
                
                while True:
                    distances = self.get_sensor_data(tof_sensors)
                    
                    if None in distances:
                        print("⚠️ 센서 데이터를 읽을 수 없습니다. 다시 시도합니다.")
                        time.sleep(1)
                        continue
                        
                    is_all_matched = (
                        abs(distances[0] - self.TARGET_DISTANCE_MM_TOF) <= self.TOLERANCE_MM_TOF and
                        abs(distances[1] - self.TARGET_DISTANCE_MM_TOF) <= self.TOLERANCE_MM_TOF and
                        abs(distances[2] - self.TARGET_DISTANCE_MM_TOF) <= self.TOLERANCE_MM_TOF and
                        abs(distances[3] - self.TARGET_DISTANCE_MM_TOF) <= self.TOLERANCE_MM_TOF
                    )
                    
                    if is_all_matched:
                        print("\n✅ 드론의 소화탄 장착부가 완벽하게 매칭되었습니다.")
                        self.pwm_fb.ChangeDutyCycle(self.SERVO_360_STOP)
                        self.pwm_lr.ChangeDutyCycle(self.SERVO_360_STOP)
                        self.load_ammo(ammo_count)
                        return True
                    else:
                        offset_fb = distances[0] - distances[2]
                        offset_lr = distances[1] - distances[3]
                        print(f"    -> 정렬 필요: 앞뒤 오차={offset_fb:.2f}mm, 좌우 오차={offset_lr:.2f}mm")
                        self.control_servos_tof(offset_fb, offset_lr)
                        
                    time.sleep(0.1)
                        
        except KeyboardInterrupt:
            print("\n\n👋 사용자 중지 요청.")
            return False
        except Exception as e:
            print(f"\n\n🚨 오류 발생: {e}")
            return False

    def process_command(self, command):
        """PC로부터 받은 명령을 처리하고 결과를 반환합니다."""
        if command.get('command') == 'start_docking':
            ammo_count = command.get('ammo_count')
            print(f"\n\n[명령 처리] '도킹 시작' 명령 수신. 장전 수: {ammo_count} 발")
            
            success = self.run_station_process(ammo_count)
            
            if success:
                return {'status': 'success', 'message': f'{ammo_count}발 장전 완료'}
            else:
                return {'status': 'failure', 'message': '도킹 및 장전 실패'}
        
        return {'status': 'error', 'message': '알 수 없는 명령'}

    def cleanup(self):
        """프로그램 종료 시 자원 정리"""
        self.pwm_fb.stop()
        self.pwm_lr.stop()
        self.pwm_ammo.stop()
        GPIO.cleanup()
        self.client.close()

    def run(self):
        """메인 실행 루프"""
        print("--- 드론 스테이션 프로그램 시작 ---")
        try:
            while True:
                if not self.client.is_connected:
                    print("서버에 재연결을 시도합니다...")
                    if self.client.connect():
                        print(">> PC로부터 명령을 기다립니다...")
                    else:
                        time.sleep(5)
                        continue

                try:
                    command = self.client.command_queue.get(timeout=1)
                    result = self.process_command(command)
                    self.client.send_response(result)
                except queue.Empty:
                    pass

        except KeyboardInterrupt:
            print("\n\n👋 사용자에 의해 프로그램이 중지됩니다.")
        finally:
            self.cleanup()
            print(">> 프로그램 종료.")


# --- 4. 메인 프로그램 진입점 ---
if __name__ == '__main__':
    # PC의 실제 로컬 이름 또는 IP 주소로 변경해주세요!
    PC_HOSTNAME = 'Your-PC-Name.local'
    station = DroneStation(pc_host=PC_HOSTNAME)
    station.run()
