# pc_rpi_server.py
import socket
import json
import threading
import time
import queue

class PC_RPi_Server:
    def __init__(self, host='0.0.0.0', port=65525):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None # 단 하나의 클라이언트 소켓만 저장
        self.client_addr = None
        self.response_q = queue.Queue()

    def start(self):
        """서버를 시작하고 클라이언트 연결을 기다리는 스레드를 시작합니다."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"\n\n>> 서버 시작. {self.host}:{self.port}에서 라즈베리파이의 연결을 기다립니다...")

        accept_thread = threading.Thread(target=self.accept_client_loop, daemon=True)
        accept_thread.start()

    def accept_client_loop(self):
        """(백그라운드 스레드) 클라이언트의 연결을 '계속해서' 수락하는 루프."""
        try:
            while True:
                client_socket, addr = self.server_socket.accept()
                print(f"\n>> {addr} 라즈베리파이가 연결되었습니다.")
                
                if self.client_socket: # 만약 이전 연결이 남아있다면 종료
                    self.client_socket.close()
                
                self.client_socket = client_socket
                self.client_addr = addr
                
                # 연결된 클라이언트로부터 응답을 수신하는 스레드를 '새로' 시작
                handler_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr), daemon=True)
                handler_thread.start()
        except OSError:
            pass

    def handle_client(self, sock, addr):
        """연결된 특정 클라이언트 소켓에서만 메시지를 처리"""
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                try:
                    response = json.loads(data.decode('utf-8').strip())
                    self.response_q.put(response)
                except json.JSONDecodeError:
                    # 필요하다면 버퍼링/구분자 기반 파싱 도입
                    continue
        except OSError:
            pass
        finally:
            print(f"\n>> {addr} 와의 연결이 끊어졌습니다.")
            # 현재 등록된 소켓이라면 초기화
            if self.client_socket is sock:
                self.client_socket = None
                self.client_addr = None

    def send_command(self, command):
        """연결된 라즈베리파이에게 명령을 전송합니다."""
        if not self.client_socket:
            print("!! 연결된 라즈베리파이가 없습니다.")
            return
            
        print(f"[명령 전송] >> {command}")
        message = json.dumps(command).encode('utf-8')
        try:
            self.client_socket.sendall(message)
        except Exception as e:
            print(f"!! 메시지 전송 실패: {e}")

    def close(self):
        """모든 소켓 연결을 정리하고 서버를 종료합니다."""
        self._running = False # 모든 스레드 루프를 중지시킴
        if self.client_socket:
            try:
                # 클라이언트에게 연결 종료를 알리기 위해 shutdown을 사용할 수 있습니다.
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except OSError:
                pass # 이미 닫혔을 수 있음
        if self.server_socket:
            self.server_socket.close()
        print(">> 서버가 종료되었습니다.")
