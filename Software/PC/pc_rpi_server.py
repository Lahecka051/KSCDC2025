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
        self.client_socket = None  # 단 하나의 클라이언트 소켓만 저장
        self.client_addr = None
        self.response_q = queue.Queue()
        # [최적화] _running 플래그 초기화 추가
        # 기존: close()에서 self._running = False를 설정하지만,
        #       __init__에서 선언하지 않아 close() 호출 전 참조 시 AttributeError 발생
        # 수정: __init__에서 True로 초기화
        self._running = True

    def start(self):
        """서버를 시작하고 클라이언트 연결을 기다리는 스레드를 시작합니다."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        self._running = True
        print(f"\n\n>> 서버 시작. {self.host}:{self.port}에서 라즈베리파이의 연결을 기다립니다...")

        accept_thread = threading.Thread(target=self.accept_client_loop, daemon=True)
        accept_thread.start()

    def accept_client_loop(self):
        """(백그라운드 스레드) 클라이언트의 연결을 '계속해서' 수락하는 루프.

        [최적화] _running 플래그로 루프 제어
        기존: while True — close() 호출 후에도 accept()에서 블로킹되어 스레드가 종료되지 않음
              (OSError를 catch하여 실질적으로는 동작하지만 의도가 불명확)
        수정: _running 플래그 체크로 종료 의도를 명시적으로 표현
        """
        try:
            while self._running:
                client_socket, addr = self.server_socket.accept()
                print(f"\n>> {addr} 라즈베리파이가 연결되었습니다.")

                if self.client_socket:
                    self.client_socket.close()

                self.client_socket = client_socket
                self.client_addr = addr

                handler_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr), daemon=True)
                handler_thread.start()
        except OSError:
            pass

    def handle_client(self, sock, addr):
        """연결된 특정 클라이언트 소켓에서만 메시지를 처리"""
        try:
            while self._running:
                data = sock.recv(4096)
                if not data:
                    break
                try:
                    response = json.loads(data.decode('utf-8').strip())
                    self.response_q.put(response)
                except json.JSONDecodeError:
                    continue
        except OSError:
            pass
        finally:
            print(f"\n>> {addr} 와의 연결이 끊어졌습니다.")
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
        self._running = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except OSError:
                pass
        if self.server_socket:
            self.server_socket.close()
        print(">> 서버가 종료되었습니다.")
