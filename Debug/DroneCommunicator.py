# DroneCommunicator.py
import socket
import json
import os

class DroneCommunicator:
    """
    PC 관제 센터와의 모든 '클라이언트' 통신을 전담하는 클래스.
    (데이터를 '보내는' 역할)
    """
    def __init__(self, pc_hostname, pc_port=65524):
        """
        통신 객체를 생성할 때 접속할 PC의 주소와 포트를 설정합니다.
        """
        self.pc_hostname = pc_hostname
        self.pc_port = pc_port
        self.s = None  # 수정: 서버 소켓 변수 초기화
        print(f"[통신 모듈] 초기화 완료. 목표 PC: {self.pc_hostname}:{self.pc_port}")

    def send_fire_report(self, coordinates, image_path):
        """PC 관제 센터로 화재 보고(좌표, 이미지)를 전송합니다."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.pc_hostname, self.pc_port))
                
                header = {
                    "type": "FIRE_REPORT",
                    "coordinates": coordinates,
                    "image_size": os.path.getsize(image_path)
                }
                header_json = json.dumps(header)
                header_bytes = header_json.encode('utf-8').ljust(1024, b' ')

                s.sendall(header_bytes)
                s.recv(1024)  # HEADER_OK
                with open(image_path, 'rb') as f:
                    s.sendall(f.read())
                
                print(f"[통신 모듈] 화재 보고 전송 성공!")
        except Exception as e:
            print(f"[통신 모듈][오류] PC로 보고 전송 실패: {e}")

    def send_status_update(self, status_message, ball_count):
        """PC 관제 센터로 간단한 상태 메시지를 전송합니다."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.pc_hostname, self.pc_port))
                
                message_data = {
                    "type": "STATUS_UPDATE",
                    "status": status_message,  # 수정: 콤마 추가
                    "ball_count": ball_count  # 수정: 키 이름 수정 (공백 제거)
                }
                message_bytes = json.dumps(message_data).encode('utf-8')
                
                s.sendall(message_bytes)
                print(f"[통신 모듈] 상태 보고 전송 성공: {status_message}")
        except Exception as e:
            print(f"[통신 모듈][오류] PC로 상태 보고 전송 실패: {e}")

    def start_receiver(self):
        host = '0.0.0.0'
        port = 65523
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 수정: self.s로 변경
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host, port))
        self.s.listen()  # 수정: s를 self.s로 변경
        print(f"Jetson 임무 대기 중... (Port: {port})")

    def receive_data(self):
        conn, addr = self.s.accept()
        with conn:  # 수정: 들여쓰기 수정
            print(f"\nPC({addr})로부터 연결됨. 명령 수신 중...")
            data = conn.recv(4096)
            return data