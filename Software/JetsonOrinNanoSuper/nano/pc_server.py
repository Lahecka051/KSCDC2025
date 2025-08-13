# PC (관제 센터)용 코드 (기능 개선판)
# 필요한 라이브러리: tkinter, tkintermapview, socket
# 설치: pip install tkintermapview

import tkinter
import tkinter.messagebox
from tkintermapview import TkinterMapView
import socket
import threading

class App:
    def __init__(self, root_widget):
        self.root = root_widget
        self.root.title("화재 감시 드론 관제 센터")
        self.root.geometry("900x650")

        # --- GUI 프레임 설정 ---
        # 지도 위젯 프레임
        self.map_frame = tkinter.Frame(self.root)
        self.map_frame.pack(fill="both", expand=True)

        # 컨트롤 패널 프레임
        self.control_frame = tkinter.Frame(self.root, pady=10)
        self.control_frame.pack(fill="x")

        # 1. 수동 좌표 입력 프레임
        self.manual_add_frame = tkinter.LabelFrame(self.control_frame, text="수동 좌표 입력")
        self.manual_add_frame.pack(side="left", padx=10, pady=5, fill="y")

        tkinter.Label(self.manual_add_frame, text="위도:").grid(row=0, column=0, padx=5, pady=2)
        self.lat_entry = tkinter.Entry(self.manual_add_frame)
        self.lat_entry.grid(row=0, column=1, padx=5, pady=2)

        tkinter.Label(self.manual_add_frame, text="경도:").grid(row=1, column=0, padx=5, pady=2)
        self.lon_entry = tkinter.Entry(self.manual_add_frame)
        self.lon_entry.grid(row=1, column=1, padx=5, pady=2)

        self.add_coord_button = tkinter.Button(self.manual_add_frame, text="좌표로 추가", command=self.add_waypoint_by_coord)
        self.add_coord_button.grid(row=0, column=2, rowspan=2, padx=10)

        # 2. 경로 관리 프레임
        self.mission_mgmt_frame = tkinter.LabelFrame(self.control_frame, text="경로 및 핑 관리")
        self.mission_mgmt_frame.pack(side="left", padx=10, pady=5, fill="y")
        
        self.clear_button = tkinter.Button(self.mission_mgmt_frame, text="마지막 지점 삭제", command=self.delete_last_waypoint)
        self.clear_button.pack(side="left", padx=10, pady=10)
        
        self.clear_all_button = tkinter.Button(self.mission_mgmt_frame, text="경로 전체 초기화", command=self.clear_all_waypoints)
        self.clear_all_button.pack(side="left", padx=10, pady=10)

        # --- 기능 추가 ---
        self.clear_fire_button = tkinter.Button(self.mission_mgmt_frame, text="화재 핑 모두 삭제", bg="lightcoral", command=self.clear_fire_pins)
        self.clear_fire_button.pack(side="left", padx=10, pady=10)

        # 3. 임무 실행 프레임
        self.mission_run_frame = tkinter.LabelFrame(self.control_frame, text="임무")
        self.mission_run_frame.pack(side="left", padx=10, pady=5, fill="y")

        self.save_button = tkinter.Button(self.mission_run_frame, text="경로 저장", command=self.save_waypoints)
        self.save_button.pack(side="left", padx=10, pady=10)

        self.start_button = tkinter.Button(self.mission_run_frame, text="순찰 시작", bg="lightblue", command=self.start_patrol)
        self.start_button.pack(side="left", padx=10, pady=10)

        # --- 지도 위젯 생성 ---
        self.map_widget = TkinterMapView(self.map_frame, width=900, height=500, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(35.1474, 129.1044) # 부산 남천동
        self.map_widget.set_zoom(15)
        self.map_widget.add_right_click_menu_command(label="여기에 경로점 추가", command=self.add_waypoint_by_click, pass_coords=True)

        # 변수 초기화
        self.waypoints = []
        self.markers = []
        self.fire_markers = [] # 화재 핑을 저장할 리스트
        
        # 데이터 수신 서버 시작
        self.start_server()

    def add_waypoint_by_click(self, coords):
        """지도 클릭으로 웨이포인트를 추가합니다."""
        self._add_waypoint_marker(coords)

    def add_waypoint_by_coord(self):
        """입력된 좌표로 웨이포인트를 추가합니다."""
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())
            self._add_waypoint_marker((lat, lon))
            self.lat_entry.delete(0, tkinter.END)
            self.lon_entry.delete(0, tkinter.END)
        except ValueError:
            tkinter.messagebox.showerror("입력 오류", "올바른 숫자 형식의 위도와 경도를 입력하세요.")

    def _add_waypoint_marker(self, coords):
        """공통 웨이포인트 추가 로직"""
        print(f"경로점 추가: {coords}")
        marker = self.map_widget.set_marker(coords[0], coords[1], text=f"WP {len(self.waypoints) + 1}")
        self.waypoints.append(coords)
        self.markers.append(marker)

    def delete_last_waypoint(self):
        """가장 마지막에 추가된 경로점을 삭제합니다."""
        if self.markers:
            last_marker = self.markers.pop()
            last_marker.delete()
            self.waypoints.pop()
            print("마지막 경로점이 삭제되었습니다.")
        else:
            tkinter.messagebox.showwarning("경고", "삭제할 경로점이 없습니다.")

    def clear_all_waypoints(self):
        """모든 경로점을 초기화합니다."""
        for marker in self.markers:
            marker.delete()
        self.waypoints.clear()
        self.markers.clear()
        print("모든 경로점이 초기화되었습니다.")

    # --- 기능 추가 ---
    def clear_fire_pins(self):
        """지도 위의 모든 화재 핑을 삭제합니다."""
        if not self.fire_markers:
            tkinter.messagebox.showinfo("정보", "삭제할 화재 핑이 없습니다.")
            return

        for marker in self.fire_markers:
            marker.delete()
        self.fire_markers.clear()
        print("모든 화재 핑이 삭제되었습니다.")


    def save_waypoints(self):
        """설정된 경로를 mission.txt 파일로 저장합니다."""
        if not self.waypoints:
            tkinter.messagebox.showwarning("경고", "저장할 경로가 없습니다.")
            return

        with open("mission.txt", "w") as f:
            f.write("QGC WPL 110\n")
            # 0번은 홈 위치이므로 1번부터 시작. 이륙 명령을 먼저 추가합니다.
            f.write(f"0\t1\t0\t22\t0\t0\t0\t0\t{self.waypoints[0][0]}\t{self.waypoints[0][1]}\t20.000000\t1\n")
            for i, (lat, lon) in enumerate(self.waypoints):
                f.write(f"{i+1}\t0\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t20.000000\t1\n")
        
        tkinter.messagebox.showinfo("성공", f"{len(self.waypoints)}개의 경로점이 'mission.txt' 파일로 저장되었습니다.")

    def start_patrol(self):
        """순찰 시작 버튼 로직"""
        if not self.waypoints:
            tkinter.messagebox.showerror("오류", "순찰을 시작하기 전에 경로를 먼저 설정하고 저장해주세요.")
            return
        # 실제 드론에 시작 신호를 보내는 로직이 여기에 들어갈 수 있습니다.
        print("--- 순찰 시작 명령 ---")
        tkinter.messagebox.showinfo("순찰 시작", "드론의 순찰을 시작합니다. (시뮬레이션)")

    def start_server(self):
        """백그라운드에서 드론의 연결을 기다리는 TCP 서버를 실행합니다."""
        server_thread = threading.Thread(target=self.run_server, daemon=True)
        server_thread.start()

    def run_server(self):
        """TCP 소켓 서버 로직"""
        host = '0.0.0.0'
        port = 4000 # 포트 번호는 그대로 9999를 사용합니다.

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # 포트 충돌 방지를 위한 소켓 옵션 설정
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()
            print(f"서버 대기 중... IP: {socket.gethostbyname(socket.gethostname())}, Port: {port}")
            
            while True: # 여러 드론의 접속을 계속 받을 수 있도록 무한 루프
                conn, addr = s.accept()
                # 각 클라이언트 연결을 별도의 스레드에서 처리
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True)
                client_thread.start()

    def handle_client(self, conn, addr):
        """클라이언트로부터 데이터를 수신하고 처리합니다."""
        print(f"{addr} 에서 연결됨.")
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    decoded_data = data.decode('utf-8')
                    lat_str, lon_str = decoded_data.split(',')
                    lat, lon = float(lat_str), float(lon_str)
                    print(f"화재 위치 수신: lat={lat}, lon={lon}")
                    
                    self.map_widget.set_position(lat, lon)
                    self.map_widget.set_zoom(17)
                    
                    # --- 기능 수정 ---
                    # 생성된 화재 마커를 리스트에 저장
                    fire_marker = self.map_widget.set_marker(lat, lon, text="!!화재 발생!!", marker_color_circle="red", marker_color_outside="red")
                    self.fire_markers.append(fire_marker)
                    
                    tkinter.messagebox.showerror("긴급!", f"화재 발생!\n위치: {lat}, {lon}")

                except Exception as e:
                    print(f"데이터 처리 오류: {e}")
        print(f"{addr} 연결 종료.")

if __name__ == "__main__":
    root = tkinter.Tk()
    app = App(root)
    root.mainloop()
