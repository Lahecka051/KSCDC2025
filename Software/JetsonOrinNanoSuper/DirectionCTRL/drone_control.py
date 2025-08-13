"""
drone_control.py
Jetson에서 UART를 통해 드론을 제어하고 GPS 데이터를 수신하는 예제
"""

import serial
import json
import time
import threading
from datetime import datetime

class JetsonDroneControl:
    """Jetson에서 드론 제어를 위한 클래스"""
    
    def __init__(self, uart_port="/dev/ttyTHS1", baudrate=115200):
        """
        초기화
        
        Args:
            uart_port (str): UART 포트
            baudrate (int): 통신 속도
        """
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.serial_conn = None
        self.seq_num = 0
        self.is_running = False
        self.rx_thread = None
        
        # 수신된 GPS 데이터 저장
        self.latest_gps_data = None
        self.gps_data_lock = threading.Lock()
        
    def connect(self):
        """UART 연결"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            # 수신 스레드 시작
            self.is_running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            
            print(f"✅ UART 연결 성공: {self.uart_port}")
            return True
            
        except Exception as e:
            print(f"❌ UART 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.is_running = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=2.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        print("UART 연결 해제됨")
    
    def _rx_loop(self):
        """수신 루프"""
        buffer = ""
        
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self._process_received_message(line)
                
            except Exception as e:
                print(f"수신 오류: {e}")
            
            time.sleep(0.01)
    
    def _process_received_message(self, message):
        """수신 메시지 처리"""
        try:
            data = json.loads(message)
            msg_type = data.get('type', '')
            
            if msg_type == 'GPS':
                # GPS 데이터 저장
                with self.gps_data_lock:
                    self.latest_gps_data = data.get('data', {})
                print(f"📍 GPS 수신: Lat={self.latest_gps_data.get('position', {}).get('lat', 'N/A'):.6f}, "
                      f"Lon={self.latest_gps_data.get('position', {}).get('lon', 'N/A'):.6f}")
                      
            elif msg_type == 'ACK':
                print(f"✅ ACK 수신: seq={data.get('seq')}")
                
            elif msg_type == 'STATUS':
                print(f"📊 상태 수신: {data.get('data')}")
                
        except json.JSONDecodeError:
            pass  # GPS 외 메시지 무시
        except Exception as e:
            print(f"메시지 처리 오류: {e}")
    
    def send_control_command(self, vertical="level", horizontal="hover", 
                           rotation=0, speed=0, speed_type="percent"):
        """
        드론 제어 명령 전송
        
        Args:
            vertical (str): 수직 방향 (up/level/down)
            horizontal (str): 수평 방향 (forward/backward/left/right/forward_left/...)
            rotation (float): 회전 각도 (0-359)
            speed (float): 속도
            speed_type (str): 속도 타입 (percent/m/s)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("❌ UART 연결되지 않음")
            return False
        
        try:
            self.seq_num += 1
            
            command = {
                'type': 'CTRL',
                'seq': self.seq_num,
                'timestamp': datetime.now().isoformat(),
                'command': {
                    'vertical': vertical,
                    'horizontal': horizontal,
                    'rotation': rotation,
                    'speed': speed,
                    'speed_type': speed_type
                }
            }
            
            json_str = json.dumps(command) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            
            print(f"📤 명령 전송: V={vertical}, H={horizontal}, R={rotation}°, S={speed}{speed_type}")
            return True
            
        except Exception as e:
            print(f"❌ 명령 전송 실패: {e}")
            return False
    
    def get_latest_gps(self):
        """최신 GPS 데이터 반환"""
        with self.gps_data_lock:
            return self.latest_gps_data.copy() if self.latest_gps_data else None
    
    def simple_control_menu(self):
        """간단한 제어 메뉴"""
        print("\n" + "="*50)
        print("드론 제어 메뉴")
        print("="*50)
        print("1. 전진 (Forward)")
        print("2. 후진 (Backward)")
        print("3. 좌측 (Left)")
        print("4. 우측 (Right)")
        print("5. 상승 (Up)")
        print("6. 하강 (Down)")
        print("7. 좌회전 (Rotate Left)")
        print("8. 우회전 (Rotate Right)")
        print("9. 대각선 이동 (Forward-Right)")
        print("0. 정지 (Hover)")
        print("q. 종료")
        print("-"*50)
    
    def run_interactive_control(self):
        """대화형 제어 실행"""
        if not self.connect():
            return
        
        print("\n드론 제어 시작. 명령을 입력하세요.")
        
        try:
            while True:
                self.simple_control_menu()
                choice = input("선택> ").strip().lower()
                
                if choice == 'q':
                    break
                elif choice == '1':
                    self.send_control_command(horizontal="forward", speed=50)
                elif choice == '2':
                    self.send_control_command(horizontal="backward", speed=50)
                elif choice == '3':
                    self.send_control_command(horizontal="left", speed=50)
                elif choice == '4':
                    self.send_control_command(horizontal="right", speed=50)
                elif choice == '5':
                    self.send_control_command(vertical="up", speed=30)
                elif choice == '6':
                    self.send_control_command(vertical="down", speed=30)
                elif choice == '7':
                    self.send_control_command(rotation=270, speed=0)  # 90도 좌회전
                elif choice == '8':
                    self.send_control_command(rotation=90, speed=0)   # 90도 우회전
                elif choice == '9':
                    self.send_control_command(horizontal="forward_right", speed=50)
                elif choice == '0':
                    self.send_control_command(horizontal="hover", speed=0)
                else:
                    print("잘못된 선택입니다.")
                
                time.sleep(0.5)  # 명령 간 간격
                
        except KeyboardInterrupt:
            print("\n\n중단됨")
        
        finally:
            self.disconnect()


def example_sequence():
    """예제 시퀀스 실행"""
    controller = JetsonDroneControl("/dev/ttyTHS1", 115200)
    
    if not controller.connect():
        return
    
    try:
        print("\n자동 비행 시퀀스 시작")
        
        # 1. 전진하면서 상승
        print("\n[1] 전진하면서 상승")
        controller.send_control_command(
            vertical="up",
            horizontal="forward",
            speed=50,
            speed_type="percent"
        )
        time.sleep(3)
        
        # 2. 우상향 대각선 이동
        print("\n[2] 우상향 대각선 이동")
        controller.send_control_command(
            vertical="level",
            horizontal="forward_right",
            speed=60,
            speed_type="percent"
        )
        time.sleep(3)
        
        # 3. 제자리 회전
        print("\n[3] 90도 우회전")
        controller.send_control_command(
            vertical="level",
            horizontal="hover",
            rotation=90,
            speed=0
        )
        time.sleep(2)
        
        # 4. 후진
        print("\n[4] 후진")
        controller.send_control_command(
            vertical="level",
            horizontal="backward",
            speed=40,
            speed_type="percent"
        )
        time.sleep(3)
        
        # 5. 하강하면서 정지
        print("\n[5] 하강하면서 정지")
        controller.send_control_command(
            vertical="down",
            horizontal="hover",
            speed=30,
            speed_type="percent"
        )
        time.sleep(3)
        
        # 6. 완전 정지
        print("\n[6] 완전 정지")
        controller.send_control_command(
            vertical="level",
            horizontal="hover",
            speed=0
        )
        
        print("\n✅ 시퀀스 완료")
        
        # GPS 데이터 확인
        time.sleep(1)
        gps = controller.get_latest_gps()
        if gps:
            print(f"\n현재 GPS 데이터:")
            print(f"  위치: {gps.get('position')}")
            print(f"  속도: {gps.get('velocity')}")
            print(f"  자세: {gps.get('attitude')}")
            print(f"  배터리: {gps.get('battery')}")
        
    except KeyboardInterrupt:
        print("\n중단됨")
    
    finally:
        controller.disconnect()


if __name__ == "__main__":
    import sys
    
    print("="*60)
    print("Jetson 드론 제어 프로그램")
    print("="*60)
    print("1. 대화형 제어 모드")
    print("2. 자동 시퀀스 실행")
    print("0. 종료")
    
    choice = input("\n선택> ").strip()
    
    if choice == "1":
        controller = JetsonDroneControl()
        controller.run_interactive_control()
    elif choice == "2":
        example_sequence()
    else:
        print("종료합니다.")
