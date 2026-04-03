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
        self.s = None
        print(f"[통신 모듈] 초기화 완료. 목표 PC: {self.pc_hostname}:{self.pc_port}")

    def send_fire_report(self, coordinates, image_path):
        """PC 관제 센터로 화재 보고(좌표, 이미지)를 전송합니다."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                # [최적화] 연결 타임아웃 설정
                # 기존: 타임아웃 없음 — PC가 응답하지 않으면 무한 대기
                # 수정: 10초 타임아웃 추가하여 네트워크 장애 시 빠른 실패
                s.settimeout(10)
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
                # [최적화] 연결 타임아웃 설정 (send_fire_report와 동일 이유)
                s.settimeout(10)
                s.connect((self.pc_hostname, self.pc_port))

                message_data = {
                    "type": "STATUS_UPDATE",
                    "status": status_message,
                    "ball_count": ball_count
                }
                message_bytes = json.dumps(message_data).encode('utf-8')

                s.sendall(message_bytes)
                print(f"[통신 모듈] 상태 보고 전송 성공: {status_message}")
        except Exception as e:
            print(f"[통신 모듈][오류] PC로 상태 보고 전송 실패: {e}")

    def start_receiver(self):
        """명령 수신 서버 시작

        [최적화] 소켓 타임아웃 및 종료 플래그 추가
        기존: accept()가 무한 블로킹 → 프로그램 종료 시 소켓이 해제되지 않음
        수정: accept 타임아웃(5초) 설정으로 주기적으로 종료 체크 가능
        """
        host = '0.0.0.0'
        port = 65523
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host, port))
        self.s.listen()
        # [최적화] accept 타임아웃 설정으로 종료 시 블로킹 방지
        self.s.settimeout(5)
        print(f"Jetson 임무 대기 중... (Port: {port})")

    def receive_data(self):
        """명령 데이터 수신

        [최적화] 타임아웃 예외 처리 추가
        기존: accept()에서 타임아웃 발생 시 예외가 상위로 전파되어 프로그램 크래시
        수정: socket.timeout 포착하여 None 반환 → 호출부에서 자연스럽게 재시도
        """
        try:
            conn, addr = self.s.accept()
            with conn:
                print(f"\nPC({addr})로부터 연결됨. 명령 수신 중...")
                data = conn.recv(4096)
                return data
        except socket.timeout:
            return None
        except OSError as e:
            print(f"[통신 모듈] 수신 오류: {e}")
            return None
