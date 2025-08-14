"""
communication.py
H743v2 FC 드론과 Jetson 간 양방향 UART 통신 모듈
GPS 데이터 송신 및 제어 명령 수신
"""

import asyncio
import serial
import json
import threading
import queue
import logging
import time
from datetime import datetime
from enum import Enum
from typing import Optional, Dict, Any, Callable, List

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

class MessageType(Enum):
    """메시지 타입"""
    GPS_DATA = "GPS"           # GPS 및 텔레메트리 데이터
    CONTROL_CMD = "CTRL"        # 드론 제어 명령
    GPS_CMD = "GPS_CMD"         # GPS 좌표 이동 명령
    ACK = "ACK"                 # 명령 수신 확인
    STATUS = "STATUS"           # 드론 상태
    ERROR = "ERROR"             # 에러 메시지
    HEARTBEAT = "HEARTBEAT"     # 연결 확인

class UARTCommunication:
    """양방향 UART 통신 클래스"""
    
    def __init__(self, uart_port="/dev/ttyTHS1", baudrate=115200):
        """
        초기화
        
        Args:
            uart_port (str): UART 포트 경로
            baudrate (int): 통신 속도
        """
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.serial_conn = None
        self.logger = logging.getLogger(__name__)
        
        # 수신 명령 큐
        self.command_queue = queue.Queue(maxsize=100)
        self.gps_command_queue = queue.Queue(maxsize=50)
        
        # GPS 데이터 버퍼
        self.latest_gps = {}
        self.gps_lock = threading.Lock()
        
        # 통신 상태
        self.is_running = False
        self.rx_thread = None
        
        # 콜백 함수들
        self.control_callback = None
        self.gps_cmd_callback = None
        self.status_callback = None
        
        # 통계
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'gps_updates': 0,
            'control_commands': 0,
            'errors': 0
        }
        
        # 하트비트
        self.last_heartbeat = time.time()
        
    def initialize(self) -> bool:
        """UART 포트 초기화"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5
            )
            
            # 버퍼 비우기
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.logger.info(f"UART 초기화 성공: {self.uart_port} @ {self.baudrate} bps")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"UART 포트 열기 실패: {e}")
            return False
            
        except Exception as e:
            self.logger.error(f"UART 초기화 실패: {e}")
            return False
    
    def start(self) -> bool:
        """통신 시작"""
        if not self.serial_conn:
            if not self.initialize():
                return False
        
        self.is_running = True
        
        # 수신 스레드 시작
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        
        self.logger.info("UART 통신 시작")
        return True
    
    def stop(self):
        """통신 중지"""
        self.is_running = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=2.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        self.logger.info(f"UART 통신 중지 - 통계: {self.stats}")
    
    def _rx_loop(self):
        """수신 루프 (별도 스레드)"""
        buffer = ""
        
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    # 데이터 읽기
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    # 줄바꿈으로 메시지 분리
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self._process_message(line)
                
                # 하트비트 체크
                if time.time() - self.last_heartbeat > 5.0:
                    self.send_heartbeat()
                
            except serial.SerialException as e:
                self.logger.error(f"시리얼 통신 오류: {e}")
                self.stats['errors'] += 1
                
            except Exception as e:
                self.logger.error(f"수신 루프 오류: {e}")
                self.stats['errors'] += 1
            
            time.sleep(0.001)  # 1ms 대기
    
    def _process_message(self, message: str):
        """수신 메시지 처리"""
        try:
            data = json.loads(message)
            msg_type = data.get('type', '')
            
            self.stats['messages_received'] += 1
            
            if msg_type == MessageType.CONTROL_CMD.value:
                # 일반 제어 명령
                self._process_control_command(data)
                
            elif msg_type == MessageType.GPS_CMD.value:
                # GPS 이동 명령
                self._process_gps_command(data)
                
            elif msg_type == MessageType.STATUS.value:
                # 상태 메시지
                if self.status_callback:
                    self.status_callback(data.get('data', {}))
                    
            elif msg_type == MessageType.HEARTBEAT.value:
                # 하트비트 응답
                self.last_heartbeat = time.time()
                self._send_message({
                    'type': MessageType.HEARTBEAT.value,
                    'timestamp': datetime.now().isoformat()
                })
                
        except json.JSONDecodeError as e:
            self.logger.warning(f"JSON 파싱 실패: {message[:50]}...")
            self.stats['errors'] += 1
            
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
            self.stats['errors'] += 1
    
    def _process_control_command(self, data: Dict[str, Any]):
        """일반 제어 명령 처리"""
        self.logger.debug(f"제어 명령 수신: {data}")
        
        # 큐에 추가
        self.command_queue.put(data)
        self.stats['control_commands'] += 1
        
        # 콜백 호출
        if self.control_callback:
            self.control_callback(data)
        
        # ACK 전송
        self.send_ack(data.get('seq', 0))
    
    def _process_gps_command(self, data: Dict[str, Any]):
        """GPS 이동 명령 처리"""
        self.logger.info(f"GPS 명령 수신: {data}")
        
        # 큐에 추가
        self.gps_command_queue.put(data)
        
        # 콜백 호출
        if self.gps_cmd_callback:
            self.gps_cmd_callback(data)
        
        # ACK 전송
        self.send_ack(data.get('seq', 0))
    
    def _send_message(self, message: Dict[str, Any]) -> bool:
        """메시지 전송"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            json_str = json.dumps(message) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            self.serial_conn.flush()
            
            self.stats['messages_sent'] += 1
            return True
            
        except Exception as e:
            self.logger.error(f"메시지 전송 실패: {e}")
            self.stats['errors'] += 1
            return False
    
    def send_gps_data(self, gps_data: Dict[str, Any]) -> bool:
        """
        GPS 데이터 전송 (드론 → Jetson)
        
        Args:
            gps_data (dict): GPS 및 텔레메트리 데이터
        """
        message = {
            'type': MessageType.GPS_DATA.value,
            'timestamp': datetime.now().isoformat(),
            'data': gps_data
        }
        
        # 최신 GPS 데이터 저장
        with self.gps_lock:
            self.latest_gps = gps_data
        
        self.stats['gps_updates'] += 1
        return self._send_message(message)
    
    def send_status(self, status_data: Dict[str, Any]) -> bool:
        """
        드론 상태 전송
        
        Args:
            status_data (dict): 상태 정보
        """
        message = {
            'type': MessageType.STATUS.value,
            'timestamp': datetime.now().isoformat(),
            'data': status_data
        }
        
        return self._send_message(message)
    
    def send_ack(self, seq_num: int):
        """ACK 전송"""
        message = {
            'type': MessageType.ACK.value,
            'seq': seq_num,
            'timestamp': datetime.now().isoformat()
        }
        
        self._send_message(message)
    
    def send_error(self, error_msg: str, error_code: int = -1):
        """에러 메시지 전송"""
        message = {
            'type': MessageType.ERROR.value,
            'error_code': error_code,
            'error_msg': error_msg,
            'timestamp': datetime.now().isoformat()
        }
        
        self._send_message(message)
    
    def send_heartbeat(self):
        """하트비트 전송"""
        message = {
            'type': MessageType.HEARTBEAT.value,
            'timestamp': datetime.now().isoformat(),
            'stats': self.stats
        }
        
        self._send_message(message)
        self.last_heartbeat = time.time()
    
    def get_control_command(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """
        일반 제어 명령 가져오기
        
        Returns:
            dict: 제어 명령 또는 None
        """
        try:
            return self.command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_gps_command(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """
        GPS 이동 명령 가져오기
        
        Returns:
            dict: GPS 명령 또는 None
        """
        try:
            return self.gps_command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def set_control_callback(self, callback: Callable):
        """제어 명령 콜백 설정"""
        self.control_callback = callback
    
    def set_gps_cmd_callback(self, callback: Callable):
        """GPS 명령 콜백 설정"""
        self.gps_cmd_callback = callback
    
    def set_status_callback(self, callback: Callable):
        """상태 메시지 콜백 설정"""
        self.status_callback = callback
    
    def get_latest_gps(self) -> Dict[str, Any]:
        """최신 GPS 데이터 반환"""
        with self.gps_lock:
            return self.latest_gps.copy()
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.serial_conn and self.serial_conn.is_open and self.is_running
    
    def get_stats(self) -> Dict[str, Any]:
        """통신 통계 반환"""
        return self.stats.copy()


# Jetson 측 통신 인터페이스
class JetsonInterface:
    """Jetson에서 드론과 통신하는 인터페이스"""
    
    def __init__(self, uart_port="/dev/ttyTHS1", baudrate=115200):
        """초기화"""
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.serial_conn = None
        self.seq_num = 0
        self.logger = logging.getLogger(__name__)
        
        # GPS 데이터 버퍼
        self.latest_gps = {}
        self.gps_lock = threading.Lock()
        
        # 수신 스레드
        self.is_running = False
        self.rx_thread = None
        
    def connect(self) -> bool:
        """연결"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            self.is_running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            
            self.logger.info(f"Jetson UART 연결: {self.uart_port}")
            return True
            
        except Exception as e:
            self.logger.error(f"연결 실패: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.is_running = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=2.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
    
    def _rx_loop(self):
        """수신 루프"""
        buffer = ""
        
        while self.is_running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line:
                            self._process_message(line)
            except:
                pass
            
            time.sleep(0.001)
    
    def _process_message(self, message: str):
        """메시지 처리"""
        try:
            data = json.loads(message)
            
            if data.get('type') == 'GPS':
                # GPS 데이터 저장
                with self.gps_lock:
                    self.latest_gps = data.get('data', {})
                    
        except:
            pass
    
    def send_control(self, cmd: List) -> bool:
        """
        드론 제어 명령 전송
        
        Args:
            cmd: 제어 명령
                - 일반: [vertical, horizontal, rotation, motor_percent]
                - GPS: ["gps", lat, lon] 또는 ["gps", lat, lon, alt]
        """
        self.seq_num += 1
        
        try:
            # GPS 명령인지 확인
            if cmd[0] == "gps":
                # GPS 이동 명령
                gps_cmd = {
                    'latitude': cmd[1],
                    'longitude': cmd[2]
                }
                if len(cmd) == 4:
                    gps_cmd['altitude'] = cmd[3]
                
                message = {
                    'type': 'GPS_CMD',
                    'seq': self.seq_num,
                    'timestamp': datetime.now().isoformat(),
                    'command': gps_cmd
                }
            else:
                # 일반 제어 명령
                message = {
                    'type': 'CTRL',
                    'seq': self.seq_num,
                    'timestamp': datetime.now().isoformat(),
                    'command': {
                        'vertical': cmd[0],
                        'horizontal': cmd[1],
                        'rotation': cmd[2],
                        'speed': cmd[3],
                        'speed_type': 'percent'
                    }
                }
            
            json_str = json.dumps(message) + '\n'
            self.serial_conn.write(json_str.encode('utf-8'))
            
            self.logger.info(f"명령 전송: {cmd}")
            return True
            
        except Exception as e:
            self.logger.error(f"명령 전송 실패: {e}")
            return False
    
    def get_gps_data(self) -> Optional[Dict[str, Any]]:
        """최신 GPS 데이터 가져오기"""
        with self.gps_lock:
            return self.latest_gps.copy() if self.latest_gps else None


# 사용 예제
if __name__ == "__main__":
    # 드론 측 사용 예제
    async def drone_example():
        uart = UARTCommunication("/dev/ttyTHS1", 115200)
        uart.start()
        
        # GPS 데이터 전송
        gps_data = {
            'position': {'lat': 35.123456, 'lon': 129.123456, 'alt_rel': 10.0},
            'velocity': {'north': 1.2, 'east': 0.5, 'down': -0.1},
            'battery': {'voltage': 12.4, 'percent': 85}
        }
        uart.send_gps_data(gps_data)
        
        # 제어 명령 수신
        cmd = uart.get_control_command()
        if cmd:
            print(f"받은 명령: {cmd}")
        
        await asyncio.sleep(10)
        uart.stop()
    
    # Jetson 측 사용 예제
    def jetson_example():
        jetson = JetsonInterface("/dev/ttyTHS1", 115200)
        jetson.connect()
        
        # 제어 명령 전송
        jetson.send_control(["level", "forward", 0, 50])
        time.sleep(1)
        
        # GPS 이동 명령
        jetson.send_control(["gps", 35.123456, 129.123456, 10.0])
        time.sleep(1)
        
        # GPS 데이터 수신
        gps = jetson.get_gps_data()
        if gps:
            print(f"GPS 데이터: {gps}")
        
        jetson.disconnect()
