"""
communication.py - 실전 최종 버전
H743v2 FC 드론과 Jetson 간 양방향 UART 통신 모듈
GPS 데이터 송신, 제어 명령 수신, 에러 처리 및 재연결 기능 포함
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
from dataclasses import dataclass, asdict

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
    CONFIG = "CONFIG"           # 설정 변경
    TELEMETRY = "TELEMETRY"     # 상세 텔레메트리

@dataclass
class ControlCommand:
    """제어 명령 데이터 클래스"""
    vertical: str = "level"
    horizontal: str = "hover"
    rotation: float = 0.0
    speed: float = 0.0
    speed_type: str = "percent"
    seq: int = 0
    timestamp: str = ""

@dataclass
class GPSCommand:
    """GPS 이동 명령 데이터 클래스"""
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    speed: float = 5.0
    seq: int = 0
    timestamp: str = ""

class UARTCommunication:
    """양방향 UART 통신 클래스 - 드론 측"""
    
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
        
        # 큐 설정
        self.command_queue = queue.Queue(maxsize=100)
        self.gps_command_queue = queue.Queue(maxsize=50)
        self.priority_queue = queue.PriorityQueue(maxsize=50)
        
        # GPS 데이터 버퍼
        self.latest_gps = {}
        self.gps_lock = threading.Lock()
        
        # 통신 상태
        self.is_running = False
        self.is_connected = False
        self.rx_thread = None
        self.monitor_thread = None
        
        # 콜백 함수들
        self.callbacks = {
            MessageType.CONTROL_CMD: None,
            MessageType.GPS_CMD: None,
            MessageType.STATUS: None,
            MessageType.CONFIG: None,
            MessageType.ERROR: None
        }
        
        # 통계
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'gps_updates': 0,
            'control_commands': 0,
            'errors': 0,
            'reconnects': 0,
            'bytes_sent': 0,
            'bytes_received': 0
        }
        
        # 하트비트
        self.last_heartbeat_sent = time.time()
        self.last_heartbeat_received = time.time()
        self.heartbeat_interval = 5.0
        self.heartbeat_timeout = 15.0
        
        # 재연결 설정
        self.auto_reconnect = True
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0
        
    def initialize(self) -> bool:
        """UART 포트 초기화"""
        try:
            # 기존 연결 닫기
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            # 새 연결 생성
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # 버퍼 비우기
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.is_connected = True
            self.reconnect_attempts = 0
            
            self.logger.info(f"UART 초기화 성공: {self.uart_port} @ {self.baudrate} bps")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"UART 포트 열기 실패: {e}")
            self.is_connected = False
            return False
            
        except Exception as e:
            self.logger.error(f"UART 초기화 실패: {e}")
            self.is_connected = False
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
        
        # 모니터링 스레드 시작
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.logger.info("UART 통신 시작")
        return True
    
    def stop(self):
        """통신 중지"""
        self.is_running = False
        self.is_connected = False
        
        # 스레드 종료 대기
        if self.rx_thread:
            self.rx_thread.join(timeout=2.0)
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        # 시리얼 포트 닫기
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except:
                pass
        
        self.logger.info(f"UART 통신 중지 - 통계: {self.stats}")
    
    def _rx_loop(self):
        """수신 루프 (별도 스레드)"""
        buffer = ""
        error_count = 0
        max_errors = 10
        
        while self.is_running:
            try:
                if not self.is_connected:
                    time.sleep(0.1)
                    continue
                
                if self.serial_conn and self.serial_conn.in_waiting:
                    # 데이터 읽기
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.stats['bytes_received'] += len(data)
                    
                    # 디코딩
                    try:
                        decoded = data.decode('utf-8', errors='ignore')
                        buffer += decoded
                    except:
                        continue
                    
                    # 메시지 파싱
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self._process_message(line)
                    
                    # 버퍼 오버플로우 방지
                    if len(buffer) > 10000:
                        self.logger.warning("수신 버퍼 오버플로우, 버퍼 초기화")
                        buffer = ""
                    
                    error_count = 0
                
            except serial.SerialException as e:
                error_count += 1
                self.logger.error(f"시리얼 통신 오류: {e}")
                self.stats['errors'] += 1
                
                if error_count >= max_errors:
                    self.logger.error("최대 오류 횟수 초과, 재연결 시도")
                    self.is_connected = False
                    self._attempt_reconnect()
                    error_count = 0
                
            except Exception as e:
                self.logger.error(f"수신 루프 오류: {e}")
                self.stats['errors'] += 1
            
            time.sleep(0.001)  # 1ms 대기
    
    def _monitor_loop(self):
        """모니터링 루프 (하트비트, 재연결 등)"""
        while self.is_running:
            try:
                current_time = time.time()
                
                # 하트비트 전송
                if current_time - self.last_heartbeat_sent > self.heartbeat_interval:
                    self.send_heartbeat()
                
                # 하트비트 타임아웃 체크
                if current_time - self.last_heartbeat_received > self.heartbeat_timeout:
                    if self.is_connected:
                        self.logger.warning("하트비트 타임아웃, 연결 상태 확인")
                        self.is_connected = False
                
                # 재연결 체크
                if not self.is_connected and self.auto_reconnect:
                    self._attempt_reconnect()
                
            except Exception as e:
                self.logger.error(f"모니터링 오류: {e}")
            
            time.sleep(1.0)
    
    def _attempt_reconnect(self):
        """재연결 시도"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            self.logger.error("최대 재연결 시도 횟수 초과")
            return
        
        self.reconnect_attempts += 1
        self.logger.info(f"재연결 시도 {self.reconnect_attempts}/{self.max_reconnect_attempts}")
        
        time.sleep(self.reconnect_delay)
        
        if self.initialize():
            self.logger.info("재연결 성공!")
            self.stats['reconnects'] += 1
            self.last_heartbeat_received = time.time()
        else:
            self.logger.error("재연결 실패")
    
    def _process_message(self, message: str):
        """수신 메시지 처리"""
        try:
            # JSON 파싱
            data = json.loads(message)
            msg_type = MessageType(data.get('type', ''))
            
            self.stats['messages_received'] += 1
            
            # 메시지 타입별 처리
            if msg_type == MessageType.CONTROL_CMD:
                self._process_control_command(data)
                
            elif msg_type == MessageType.GPS_CMD:
                self._process_gps_command(data)
                
            elif msg_type == MessageType.STATUS:
                self._process_status(data)
                
            elif msg_type == MessageType.HEARTBEAT:
                self.last_heartbeat_received = time.time()
                self._send_message({
                    'type': MessageType.HEARTBEAT.value,
                    'timestamp': datetime.now().isoformat(),
                    'echo': True
                })
                
            elif msg_type == MessageType.CONFIG:
                self._process_config(data)
                
            elif msg_type == MessageType.ERROR:
                self.logger.error(f"에러 메시지 수신: {data.get('error_msg')}")
                
        except json.JSONDecodeError as e:
            self.logger.debug(f"JSON 파싱 실패: {message[:50]}...")
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
        if self.callbacks[MessageType.CONTROL_CMD]:
            self.callbacks[MessageType.CONTROL_CMD](data)
        
        # ACK 전송
        self.send_ack(data.get('seq', 0), True)
    
    def _process_gps_command(self, data: Dict[str, Any]):
        """GPS 이동 명령 처리"""
        self.logger.info(f"GPS 명령 수신: {data}")
        
        # 큐에 추가
        self.gps_command_queue.put(data)
        
        # 콜백 호출
        if self.callbacks[MessageType.GPS_CMD]:
            self.callbacks[MessageType.GPS_CMD](data)
        
        # ACK 전송
        self.send_ack(data.get('seq', 0), True)
    
    def _process_status(self, data: Dict[str, Any]):
        """상태 메시지 처리"""
        if self.callbacks[MessageType.STATUS]:
            self.callbacks[MessageType.STATUS](data.get('data', {}))
    
    def _process_config(self, data: Dict[str, Any]):
        """설정 메시지 처리"""
        config = data.get('config', {})
        
        # 설정 적용
        if 'heartbeat_interval' in config:
            self.heartbeat_interval = config['heartbeat_interval']
        
        if 'auto_reconnect' in config:
            self.auto_reconnect = config['auto_reconnect']
        
        # 콜백 호출
        if self.callbacks[MessageType.CONFIG]:
            self.callbacks[MessageType.CONFIG](config)
        
        self.logger.info(f"설정 변경: {config}")
    
    def _send_message(self, message: Dict[str, Any]) -> bool:
        """메시지 전송 (내부용)"""
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            # JSON 인코딩
            json_str = json.dumps(message, separators=(',', ':')) + '\n'
            data = json_str.encode('utf-8')
            
            # 전송
            bytes_written = self.serial_conn.write(data)
            self.serial_conn.flush()
            
            self.stats['messages_sent'] += 1
            self.stats['bytes_sent'] += bytes_written
            
            return True
            
        except Exception as e:
            self.logger.error(f"메시지 전송 실패: {e}")
            self.stats['errors'] += 1
            self.is_connected = False
            return False
    
    def send_gps_data(self, gps_data: Dict[str, Any]) -> bool:
        """GPS 데이터 전송"""
        message = {
            'type': MessageType.GPS_DATA.value,
            'timestamp': datetime.now().isoformat(),
            'data': gps_data
        }
        
        # 최신 GPS 저장
        with self.gps_lock:
            self.latest_gps = gps_data
        
        self.stats['gps_updates'] += 1
        return self._send_message(message)
    
    def send_telemetry(self, telemetry_data: Dict[str, Any]) -> bool:
        """상세 텔레메트리 전송"""
        message = {
            'type': MessageType.TELEMETRY.value,
            'timestamp': datetime.now().isoformat(),
            'data': telemetry_data
        }
        
        return self._send_message(message)
    
    def send_status(self, status_data: Dict[str, Any]) -> bool:
        """드론 상태 전송"""
        message = {
            'type': MessageType.STATUS.value,
            'timestamp': datetime.now().isoformat(),
            'data': status_data
        }
        
        return self._send_message(message)
    
    def send_ack(self, seq_num: int, success: bool = True):
        """ACK 전송"""
        message = {
            'type': MessageType.ACK.value,
            'seq': seq_num,
            'success': success,
            'timestamp': datetime.now().isoformat()
        }
        
        self._send_message(message)
    
    def send_error(self, error_msg: str, error_code: int = -1, details: Dict = None):
        """에러 메시지 전송"""
        message = {
            'type': MessageType.ERROR.value,
            'error_code': error_code,
            'error_msg': error_msg,
            'details': details or {},
            'timestamp': datetime.now().isoformat()
        }
        
        self._send_message(message)
    
    def send_heartbeat(self):
        """하트비트 전송"""
        message = {
            'type': MessageType.HEARTBEAT.value,
            'timestamp': datetime.now().isoformat(),
            'stats': self.stats,
            'connected': self.is_connected
        }
        
        if self._send_message(message):
            self.last_heartbeat_sent = time.time()
    
    def get_control_command(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """일반 제어 명령 가져오기"""
        try:
            return self.command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_gps_command(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """GPS 이동 명령 가져오기"""
        try:
            return self.gps_command_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_priority_command(self, timeout: float = 0.1) -> Optional[Tuple[int, Any]]:
        """우선순위 명령 가져오기"""
        try:
            return self.priority_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def set_callback(self, msg_type: MessageType, callback: Callable):
        """콜백 함수 설정"""
        self.callbacks[msg_type] = callback
    
    def get_latest_gps(self) -> Dict[str, Any]:
        """최신 GPS 데이터 반환"""
        with self.gps_lock:
            return self.latest_gps.copy()
    
    def get_stats(self) -> Dict[str, Any]:
        """통신 통계 반환"""
        return self.stats.copy()
    
    def reset_stats(self):
        """통계 초기화"""
        for key in self.stats:
            if key != 'reconnects':
                self.stats[key] = 0
    
    def is_healthy(self) -> bool:
        """통신 상태 정상 여부"""
        if not self.is_connected:
            return False
        
        # 최근 하트비트 확인
        if time.time() - self.last_heartbeat_received > self.heartbeat_timeout:
            return False
        
        # 에러율 확인
        if self.stats['messages_received'] > 100:
            error_rate = self.stats['errors'] / self.stats['messages_received']
            if error_rate > 0.1:  # 10% 이상 에러
                return False
        
        return True
