"""
drone_communication.py
GPS 데이터 송신과 드론 제어 명령 수신을 하나의 UART로 처리하는 통합 통신 모듈
"""

import asyncio
import serial
import json
import threading
import queue
import logging
from datetime import datetime
from enum import Enum

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

class MessageType(Enum):
    """메시지 타입 구분"""
    GPS_DATA = "GPS"
    CONTROL_CMD = "CTRL"
    ACK = "ACK"
    ERROR = "ERR"
    STATUS = "STATUS"

class UnifiedUARTComm:
    """통합 UART 통신 클래스"""
    
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
        self.logger = logging.getLogger(__name__)
        
        # 수신 명령 큐
        self.command_queue = queue.Queue()
        
        # 통신 상태
        self.is_running = False
        self.rx_thread = None
        self.tx_task = None
        
        # 콜백 함수
        self.control_callback = None
        
    def init_uart(self):
        """UART 초기화"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # Non-blocking read
            )
            self.logger.info(f"UART 초기화 성공: {self.uart_port} @ {self.baudrate}")
            return True
        except Exception as e:
            self.logger.error(f"UART 초기화 실패: {e}")
            return False
    
    def start_communication(self):
        """통신 시작"""
        if not self.serial_conn:
            if not self.init_uart():
                return False
        
        self.is_running = True
        
        # 수신 스레드 시작
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        
        self.logger.info("통합 UART 통신 시작")
        return True
    
    def stop_communication(self):
        """통신 중지"""
        self.is_running = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=2.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        self.logger.info("통합 UART 통신 중지")
    
    def _rx_loop(self):
        """수신 루프 (별도 스레드에서 실행)"""
        buffer = ""
        
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    # 데이터 읽기
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 줄바꿈 기준으로 메시지 파싱
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self._process_received_message(line)
                
            except Exception as e:
                self.logger.error(f"수신 오류: {e}")
            
            # CPU 사용률 감소를 위한 짧은 대기
            threading.Event().wait(0.01)
    
    def _process_received_message(self, message):
        """수신된 메시지 처리"""
        try:
            # JSON 파싱
            data = json.loads(message)
            msg_type = data.get('type', '')
            
            if msg_type == MessageType.CONTROL_CMD.value:
                # 제어 명령 처리
                self.logger.info(f"제어 명령 수신: {data}")
                
                # 명령을 큐에 추가
                self.command_queue.put(data)
                
                # 콜백 함수 호출
                if self.control_callback:
                    self.control_callback(data)
                
                # ACK 전송
                self.send_ack(data.get('seq', 0))
                
            else:
                self.logger.debug(f"기타 메시지 수신: {data}")
                
        except json.JSONDecodeError:
            self.logger.error(f"JSON 파싱 실패: {message}")
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
    
    def send_gps_data(self, gps_data):
        """
        GPS 데이터 전송
        
        Args:
            gps_data (dict): GPS 및 텔레메트리 데이터
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            # 메시지 구성
            message = {
                'type': MessageType.GPS_DATA.value,
                'timestamp': datetime.now().isoformat(),
                'data': gps_data
            }
            
            # JSON 변환 및 전송
            json_str = json.dumps(message) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            
            self.logger.debug(f"GPS 데이터 전송: {len(json_str)} bytes")
            return True
            
        except Exception as e:
            self.logger.error(f"GPS 데이터 전송 실패: {e}")
            return False
    
    def send_status(self, status_data):
        """
        드론 상태 전송
        
        Args:
            status_data (dict): 드론 상태 정보
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            message = {
                'type': MessageType.STATUS.value,
                'timestamp': datetime.now().isoformat(),
                'data': status_data
            }
            
            json_str = json.dumps(message) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            
            return True
            
        except Exception as e:
            self.logger.error(f"상태 전송 실패: {e}")
            return False
    
    def send_ack(self, seq_num):
        """
        ACK 메시지 전송
        
        Args:
            seq_num (int): 시퀀스 번호
        """
        try:
            message = {
                'type': MessageType.ACK.value,
                'seq': seq_num,
                'timestamp': datetime.now().isoformat()
            }
            
            json_str = json.dumps(message) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            
        except Exception as e:
            self.logger.error(f"ACK 전송 실패: {e}")
    
    def get_control_command(self, timeout=0.1):
        """
        제어 명령 가져오기 (큐에서)
        
        Args:
            timeout (float): 타임아웃 (초)
            
        Returns:
            dict: 제어 명령 또는 None
        """
        try:
            return self.command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def set_control_callback(self, callback):
        """
        제어 명령 수신 콜백 설정
        
        Args:
            callback (function): 콜백 함수
        """
        self.control_callback = callback


class DroneUARTInterface:
    """드론 제어를 위한 UART 인터페이스"""
    
    def __init__(self, drone_connection, drone_controller):
        """
        초기화
        
        Args:
            drone_connection: DroneConnection 인스턴스
            drone_controller: DroneController 인스턴스
        """
        self.connection = drone_connection
        self.controller = drone_controller
        self.uart_comm = UnifiedUARTComm()
        self.logger = logging.getLogger(__name__)
        
        # GPS 스트리밍 태스크
        self.gps_task = None
        self.gps_streaming = False
        
        # 제어 명령 처리 태스크
        self.control_task = None
        
        # 제어 명령 콜백 설정
        self.uart_comm.set_control_callback(self._on_control_command)
    
    async def start(self):
        """인터페이스 시작"""
        # UART 통신 시작
        if not self.uart_comm.start_communication():
            return False
        
        # GPS 스트리밍 시작
        self.gps_streaming = True
        self.gps_task = asyncio.create_task(self._gps_streaming_loop())
        
        # 제어 명령 처리 시작
        self.control_task = asyncio.create_task(self._control_processing_loop())
        
        self.logger.info("드론 UART 인터페이스 시작")
        return True
    
    async def stop(self):
        """인터페이스 중지"""
        self.gps_streaming = False
        
        if self.gps_task:
            await self.gps_task
        
        if self.control_task:
            self.control_task.cancel()
            try:
                await self.control_task
            except asyncio.CancelledError:
                pass
        
        self.uart_comm.stop_communication()
        self.logger.info("드론 UART 인터페이스 중지")
    
    async def _gps_streaming_loop(self):
        """GPS 데이터 스트리밍 루프"""
        while self.gps_streaming:
            try:
                if self.connection.is_connected:
                    # 텔레메트리 데이터 수집
                    gps_data = {}
                    drone = self.connection.get_drone_instance()
                    
                    # 위치 정보
                    async for position in drone.telemetry.position():
                        gps_data['position'] = {
                            'lat': position.latitude_deg,
                            'lon': position.longitude_deg,
                            'alt_abs': position.absolute_altitude_m,
                            'alt_rel': position.relative_altitude_m
                        }
                        break
                    
                    # 속도 정보
                    async for velocity in drone.telemetry.velocity_ned():
                        gps_data['velocity'] = {
                            'north': velocity.north_m_s,
                            'east': velocity.east_m_s,
                            'down': velocity.down_m_s
                        }
                        break
                    
                    # 자세 정보
                    async for attitude in drone.telemetry.attitude_euler():
                        gps_data['attitude'] = {
                            'roll': attitude.roll_deg,
                            'pitch': attitude.pitch_deg,
                            'yaw': attitude.yaw_deg
                        }
                        break
                    
                    # GPS 상태
                    async for gps_info in drone.telemetry.gps_info():
                        gps_data['gps_status'] = {
                            'satellites': gps_info.num_satellites,
                            'fix_type': gps_info.fix_type.value
                        }
                        break
                    
                    # 배터리 상태
                    async for battery in drone.telemetry.battery():
                        gps_data['battery'] = {
                            'voltage': battery.voltage_v,
                            'percent': battery.remaining_percent
                        }
                        break
                    
                    # UART로 전송
                    self.uart_comm.send_gps_data(gps_data)
                
            except Exception as e:
                self.logger.error(f"GPS 스트리밍 오류: {e}")
            
            # 100ms 간격
            await asyncio.sleep(0.1)
    
    async def _control_processing_loop(self):
        """제어 명령 처리 루프"""
        while True:
            try:
                # 제어 명령 확인
                cmd = self.uart_comm.get_control_command(timeout=0.1)
                
                if cmd:
                    await self._process_control_command(cmd)
                
                await asyncio.sleep(0.01)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"제어 처리 오류: {e}")
    
    def _on_control_command(self, cmd_data):
        """제어 명령 수신 콜백"""
        self.logger.info(f"제어 명령 콜백: {cmd_data}")
    
    async def _process_control_command(self, cmd_data):
        """
        제어 명령 처리
        
        예상 명령 형식:
        {
            'type': 'CTRL',
            'seq': 123,
            'command': {
                'vertical': 'up/level/down',
                'horizontal': 'forward/backward/left/right/forward_left/...',
                'rotation': 0-359,
                'speed': 0-100 (percent) or m/s,
                'speed_type': 'percent' or 'm/s'
            }
        }
        """
        try:
            command = cmd_data.get('command', {})
            
            # 제어 명령 실행
            success = await self.controller.send_control_command(
                vertical=command.get('vertical', 'level'),
                horizontal=command.get('horizontal', 'hover'),
                rotation_deg=command.get('rotation', 0),
                speed=command.get('speed', 0),
                speed_type=command.get('speed_type', 'percent')
            )
            
            # 상태 전송
            status = {
                'command_executed': success,
                'seq': cmd_data.get('seq', 0)
            }
            self.uart_comm.send_status(status)
            
        except Exception as e:
            self.logger.error(f"명령 처리 실패: {e}")


# 사용 예제
async def main():
    """통합 UART 통신 사용 예제"""
    from drone_connection import DroneConnection
    from drone_control import DroneController
    
    # 드론 연결
    connection = DroneConnection("serial:///dev/ttyTHS0:115200")
    
    if await connection.connect():
        controller = DroneController(connection)
        
        # UART 인터페이스 생성 및 시작
        uart_interface = DroneUARTInterface(connection, controller)
        await uart_interface.start()
        
        # 드론 시동 및 이륙
        await connection.arm()
        await asyncio.sleep(2)
        await connection.takeoff(altitude=2.0)
        await asyncio.sleep(5)
        
        # Offboard 모드 시작
        await controller.start_offboard_mode()
        
        print("UART 통신 활성화됨. GPS 데이터 송신 및 제어 명령 수신 중...")
        print("Jetson에서 제어 명령을 보내면 자동으로 실행됩니다.")
        
        # 메인 루프 (Ctrl+C로 종료)
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n종료 중...")
        
        # 정리
        await controller.stop_offboard_mode()
        await connection.land()
        await connection.disarm()
        await uart_interface.stop()
    
    else:
        print("드론 연결 실패!")


if __name__ == "__main__":
    asyncio.run(main())
