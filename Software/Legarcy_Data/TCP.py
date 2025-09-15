# server.py (Jetson Orin Nano)
import socket

# 서버 IP와 포트 (Jetson의 IP로 변경)
HOST = '0.0.0.0'  # 모든 인터페이스에서 접속 허용
PORT = 5000

# 소켓 생성
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"[서버] {PORT} 포트에서 연결 대기 중...")

conn, addr = server_socket.accept()
print(f"[서버] 연결됨: {addr}")

while True:
    data = conn.recv(1024).decode()
    if not data:
        break
    print(f"[서버] 받은 데이터: {data}")

    # 응답 전송
    reply = input("[서버] 보낼 메시지 입력: ")
    conn.sendall(reply.encode())

conn.close()
server_socket.close()
