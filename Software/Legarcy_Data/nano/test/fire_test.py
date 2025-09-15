# Jetson Orin Nano (드론)용 시뮬레이션 코드 (경로 수신 기능 추가)
# PC로부터 순찰 경로를 전송받아 시뮬레이션을 시작합니다.

import socket
import threading
import time
import json

# --- 설정 변수 ---
# PC 관제 센터의 IP 주소 (화재 경보를 보낼 목적지)
CONTROLLER_PC_IP = '192.168.83.118'  # 예: '192.168.0.5'

# --- 순찰 시뮬레이션 로직 ---
def run_patrol_simulation(path):
    """전송받은 경로(path)에 따라 순찰 시뮬레이션을 수행합니다."""
    print("\n--- 전송받은 경로로 순찰 시작 ---")
    if not path:
        print("경로 정보가 비어있어 순찰을 시작할 수 없습니다.")
        return

    for i, waypoint in enumerate(path):
        lat, lon = waypoint['lat'], waypoint['lon']
        print(f"경로점 {i+1} ({lat:.4f}, {lon:.4f}) 으로 이동 중...")
        time.sleep(5) # 각 경로점까지 5초간 비행한다고 가정
        print(f"-> 경로점 {i+1} 도착.")

    print("\n최종 목적지 도착. 주변을 스캔합니다...")
    time.sleep(3)

    # 마지막 경로점 위치에서 화재가 발생했다고 가정
    print("!!! 가상 화재 탐지 !!!")
    fire_location = path[-1]
    fire_lat, fire_lon = fire_location['lat'], fire_location['lon']
    
    print(f"화재 위치({fire_lat:.4f}, {fire_lon:.4f})를 관제 센터로 전송합니다.")
    send_fire_alert(fire_lat, fire_lon)
    print("--- 순찰 임무 완료 ---")

def send_fire_alert(lat, lon):
    """화재 발생 위치를 PC 관제 센터로 전송합니다 (포트: 4000)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROLLER_PC_IP, 4000))
            message = f"{lat},{lon}"
            s.sendall(message.encode('utf-8'))
            print("화재 경보 데이터 전송 성공!")
    except Exception as e:
        print(f"[오류] 화재 경보 전송 실패: {e}")

# --- 메인 로직: PC로부터 순찰 명령 수신 대기 ---
def main():
    """PC로부터 순찰 명령(경로 데이터)을 수신 대기하는 서버를 실행합니다."""
    host = '0.0.0.0'  # 모든 인터페이스에서 연결 허용
    port = 3999       # 임무 수신용 포트는 3999 사용

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen()
        print(f"Jetson 시뮬레이터: 순찰 명령 대기 중... (Port: {port})")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"\nPC({addr})로부터 연결됨. 경로 데이터 수신 중...")
                data = conn.recv(4096) # 경로 데이터가 길 수 있으므로 넉넉하게 받음
                if data:
                    try:
                        # 수신된 JSON 데이터를 파이썬 리스트로 변환
                        path_data = json.loads(data.decode('utf-8'))
                        # 별도의 스레드에서 순찰 시뮬레이션 실행
                        patrol_thread = threading.Thread(target=run_patrol_simulation, args=(path_data,), daemon=True)
                        patrol_thread.start()
                    except json.JSONDecodeError:
                        print("수신된 데이터가 올바른 JSON 형식이 아닙니다.")
                    except Exception as e:
                        print(f"경로 데이터 처리 중 오류 발생: {e}")

if __name__ == '__main__':
    main()

