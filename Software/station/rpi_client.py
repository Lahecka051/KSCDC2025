# rpi_client.py
import socket
import json
import time
import threading
import queue

class RPiClient:
    def __init__(self, host, port=65525):
        """클라이언트 객체를 초기화합니다."""
        self.host = host
        self.port = port
        self.client_socket = None
        self.is_connected = False
        # PC로부터 받은 명령을 저장할 큐
        self.command_queue = queue.Queue()

    def connect(self):
        """서버에 연결을 시도합니다."""
        try:
            # 소켓을 새로 생성하여 연결
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            self.is_connected = True
            print("\n>> 서버에 성공적으로 연결되었습니다.")
            
            # 메시지 수신을 위한 별도 스레드 시작
            receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
            receiver_thread.start()
            return True
        except Exception as e:
            # print(f"!! 서버 연결 실패: {e}") # 재시도 중에는 메시지를 생략할 수 있음
            return False

    def _receive_loop(self):
        """(백그라운드 스레드) 서버로부터 명령을 계속 수신하여 '명령 큐'에 넣습니다."""
        while self.is_connected:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    break
                command = json.loads(data.decode('utf-8').strip())
                print(f"\n[명령 수신] -> 큐에 저장: {command}")
                self.command_queue.put(command) # 받은 명령을 큐에 넣음
            except:
                break # 오류 발생 시 루프 종료
        
        # 연결이 끊어졌음을 알림
        self.is_connected = False
        print("\n>> 서버와의 연결이 끊어졌습니다. 5초 후 재연결을 시도합니다...")

    def send_response(self, response_data):
        """처리 결과를 PC(서버)로 다시 전송합니다."""
        if not self.is_connected:
            return
        try:
            message = json.dumps(response_data).encode('utf-8')
            self.client_socket.sendall(message)
        except Exception as e:
            print(f"!! 응답 전송 실패: {e}")

    def close(self):
        self.is_connected = False
        if self.client_socket:
            self.client_socket.close()


# --- 메인 실행 로직 ---
if __name__ == "__main__":
    # ❗️❗️❗️PC의 실제 로컬 이름 또는 IP 주소로 반드시 변경해주세요❗️❗️❗️
    PC_HOSTNAME = 'Your-PC-Name.local'
    client = RPiClient(host=PC_HOSTNAME)

    try:
        while True:
            # ✅ 연결 상태를 확인하고, 끊어져 있다면 재연결 시도
            if not client.is_connected:
                if client.connect():
                    print(">> PC로부터 명령을 기다립니다...")
                else:
                    print("연결 실패 5초 후 다시 시도합니다.")
                    time.sleep(5) # 연결 실패 시 5초 후 다시 시도
                    continue

            # ✅ 큐에서 명령이 올 때까지 기다림 (비어있다면 다른 작업 가능)
            try:
                command = client.command_queue.get(timeout=1) # 1초간 대기
                result = process_command(command)
                client.send_response(result)
            except queue.Empty:
                # 1초 동안 큐에 아무것도 없으면 이 부분이 실행됨
                # print(".", end="", flush=True) # 대기 중임을 표시
                pass

    except KeyboardInterrupt:
        print("\n>> 사용자에 의해 프로그램이 중지됩니다.")
    finally:
        client.close()

    print(">> 클라이언트 프로그램 종료.")
