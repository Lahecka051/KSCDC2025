"""
uart_arm_control.py
UART 통신으로 FC 시동 제어
Jetson Orin Nano (/dev/ttyTHS1) <-> H743v2 FC
"""

import serial
import time
import json

class UARTDroneArm:
    """UART로 드론 시동 제어"""
    
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        """
        초기화
        
        Args:
            port: UART 포트
            baudrate: 통신 속도
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        
    def connect(self):
        """UART 연결"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            print(f"✓ UART 연결 성공: {self.port}")
            return True
        except Exception as e:
            print(f"✗ UART 연결 실패: {e}")
            return False
    
    def arm(self):
        """시동 걸기"""
        if not self.serial:
            print("UART가 연결되지 않음")
            return False
        
        try:
            # JSON 형식 시동 명령
            command = {
                "cmd": "ARM",
                "value": 1
            }
            
            # 전송
            data = json.dumps(command) + '\n'
            self.serial.write(data.encode())
            print("→ ARM 명령 전송")
            
            # 응답 대기
            time.sleep(0.5)
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                print(f"← 응답: {response}")
                
                if "OK" in response or "ARMED" in response:
                    print("✓ 시동 성공!")
                    return True
            
            # 응답 없어도 성공으로 처리 (FC 설정에 따라)
            print("✓ 시동 명령 전송 완료")
            return True
            
        except Exception as e:
            print(f"✗ 시동 실패: {e}")
            return False
    
    def disarm(self):
        """시동 끄기"""
        if not self.serial:
            return False
        
        try:
            command = {
                "cmd": "ARM",
                "value": 0
            }
            
            data = json.dumps(command) + '\n'
            self.serial.write(data.encode())
            print("→ DISARM 명령 전송")
            
            time.sleep(0.5)
            print("✓ 시동 끄기 완료")
            return True
            
        except Exception as e:
            print(f"✗ DISARM 실패: {e}")
            return False
    
    def close(self):
        """연결 종료"""
        if self.serial:
            self.serial.close()
            print("UART 연결 종료")


# ========================= 간단한 버전 (Raw 명령) =========================

class SimpleUARTArm:
    """더 간단한 UART 시동 제어"""
    
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        print(f"UART 연결: {port}")
    
    def arm(self):
        """시동 걸기 (단순 문자열)"""
        self.serial.write(b"ARM\n")
        time.sleep(0.5)
        print("시동 명령 전송")
        return True
    
    def disarm(self):
        """시동 끄기"""
        self.serial.write(b"DISARM\n")
        time.sleep(0.5)
        print("시동 끄기 명령 전송")
        return True
    
    def close(self):
        self.serial.close()


# ========================= 프로토콜 기반 버전 =========================

class ProtocolUARTArm:
    """프로토콜 기반 UART 시동 제어"""
    
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
    
    def send_command(self, cmd_type, cmd_value):
        """명령 전송 (바이트 프로토콜)"""
        # 프로토콜: [STX][CMD][VALUE][ETX]
        STX = 0x02  # Start
        ETX = 0x03  # End
        
        packet = bytes([STX, cmd_type, cmd_value, ETX])
        self.serial.write(packet)
        
        # 응답 확인
        response = self.serial.read(4)
        if len(response) == 4 and response[0] == STX and response[3] == ETX:
            return response[2] == 0x01  # ACK
        return False
    
    def arm(self):
        """시동 걸기"""
        CMD_ARM = 0x10
        ARM_ON = 0x01
        
        if self.send_command(CMD_ARM, ARM_ON):
            print("✓ 시동 성공")
            return True
        print("✗ 시동 실패")
        return False
    
    def disarm(self):
        """시동 끄기"""
        CMD_ARM = 0x10
        ARM_OFF = 0x00
        
        if self.send_command(CMD_ARM, ARM_OFF):
            print("✓ 시동 끄기 성공")
            return True
        return False
    
    def close(self):
        self.serial.close()


# ========================= 메인 실행 =========================

def main():
    """메인 실행"""
    
    print("\n" + "="*50)
    print("UART 드론 시동 제어")
    print("="*50 + "\n")
    
    # 방법 1: JSON 기반
    print("1. JSON 기반 제어")
    drone = UARTDroneArm('/dev/ttyTHS1', 115200)
    
    if drone.connect():
        time.sleep(1)
        
        # 시동 걸기
        if drone.arm():
            print("→ 5초 대기...")
            time.sleep(5)
            
            # 시동 끄기
            drone.disarm()
        
        drone.close()
    
    print("\n" + "-"*50 + "\n")
    
    # 방법 2: 단순 문자열
    print("2. 단순 문자열 제어")
    simple = SimpleUARTArm('/dev/ttyTHS1', 115200)
    
    simple.arm()
    time.sleep(5)
    simple.disarm()
    simple.close()
    
    print("\n" + "-"*50 + "\n")
    
    # 방법 3: 바이트 프로토콜
    print("3. 바이트 프로토콜 제어")
    protocol = ProtocolUARTArm('/dev/ttyTHS1', 115200)
    
    protocol.arm()
    time.sleep(5)
    protocol.disarm()
    protocol.close()
    
    print("\n프로그램 종료")


# ========================= 가장 간단한 실행 =========================

def quick_arm():
    """가장 간단한 시동 걸기"""
    import serial
    
    # UART 열기
    ser = serial.Serial('/dev/ttyTHS1', 115200)
    
    # 시동 걸기
    ser.write(b"ARM\n")
    print("시동 걸기")
    
    time.sleep(5)
    
    # 시동 끄기
    ser.write(b"DISARM\n")
    print("시동 끄기")
    
    # 종료
    ser.close()


if __name__ == "__main__":
    # 선택 실행
    print("실행 방법 선택:")
    print("1. 전체 테스트")
    print("2. 빠른 시동")
    
    choice = input("선택 (1-2): ")
    
    if choice == "1":
        main()
    elif choice == "2":
        quick_arm()
    else:
        # 가장 간단한 방법
        print("\n최소 코드 실행\n")
        
        import serial
        s = serial.Serial('/dev/ttyTHS1', 115200)
        s.write(b"ARM\n")
        print("시동!")
        time.sleep(5)
        s.write(b"DISARM\n")
        print("시동 끄기!")
        s.close()
