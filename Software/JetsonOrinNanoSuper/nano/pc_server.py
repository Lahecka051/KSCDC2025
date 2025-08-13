# PC (관제 센터)용 코드

import tkinter
import tkinter.messagebox
from tkintermapview import TkinterMapView
import socket
import threading
import json

class App:
    def __init__(self, root_widget):
        self.root = root_widget
        self.root.title("화재 감시 드론 관제 센터")
        self.root.geometry("900x650")

        # --- GUI 프레임 설정 ---
        self.map_frame = tkinter.Frame(self.root)
        self.map_frame.pack(fill="both", expand=True)

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

        # 2. 경로 및 핑 관리 프레임
        self.mission_mgmt_frame = tkinter.LabelFrame(self.control_frame, text="경로 및 핑 관리")
        self.mission_mgmt_frame.pack(side="left", padx=10, pady=5, fill="y")
        self.clear_button = tkinter.Button(self.mission_mgmt_frame, text="마지막 지점 삭제", command=self.delete_last_waypoint)
        self.clear_button.pack(side="left", padx=10, pady=10)
        self.clear_all_button = tkinter.Button(self.mission_mgmt_frame, text="경로 전체 초기화", command=self.clear_all_waypoints)
        self.clear_all_button.pack(side="left", padx=10, pady=10)
        self.clear_fire_button = tkinter.Button(self.mission_mgmt_frame, text="화재 핑 모두 삭제", bg="lightcoral", command=self.clear_fire_pins)
        self.clear_fire_button.pack(side="left", padx=10, pady=10)

        # 3. 드론 연결 및 임무 프레임
        self.mission_run_frame = tkinter.LabelFrame(self.control_frame, text="임무")
        self.mission_run_frame.pack(side="left", padx=10, pady=5, fill="y")
        tkinter.Label(self.mission_run_frame, text="드론 IP:").pack(side="left", padx=5)
        self.drone_ip_entry = tkinter.Entry(self.mission_run_frame)
        self.drone_ip_entry.pack(side="left", padx=5)
        self.start_button = tkinter.Button(self.mission_run_frame, text="순찰 시작", bg="lightblue", command=self.start_patrol)
        self.start_button.pack(side="left", padx=10, pady=10)

        # --- 지도 위젯 및 변수 초기화 ---
        self.map_widget = TkinterMapView(self.map_frame, width=900, height=500, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(35.1422, 129.0968)
        self.map_widget.set_zoom(15)
        self.map_widget.add_right_click_menu_command(label="여기에 경로점 추가", command=self.add_waypoint_by_click, pass_coords=True)
        self.waypoints = []
        self.markers = []
        self.fire_markers = []
        
        # 화재 경보 수신 서버 시작 (포트: 9999)
        self.start_alert_server()

    def start_patrol(self):
        """설정된 경로를 드론으로 전송하고 순찰 시작을 명령합니다."""
        drone_ip = self.drone_ip_entry.get()
        if not drone_ip:
            tkinter.messagebox.showerror("오류", "드론의 IP 주소를 입력해주세요.")
            return
        if not self.waypoints:
            tkinter.messagebox.showerror("오류", "순찰을 시작하기 전에 경로를 먼저 설정해주세요.")
            return

        # 경로 데이터를 JSON으로 변환
        path_data = [{"lat": lat, "lon": lon} for lat, lon in self.waypoints]
        message = json.dumps(path_data)

        # GUI가 멈추지 않도록 별도의 스레드에서 데이터 전송
        thread = threading.Thread(target=self.send_command_to_drone, args=(drone_ip, message), daemon=True)
        thread.start()

    def send_command_to_drone(self, drone_ip, message):
        """드론에 순찰 경로 데이터를 전송합니다 (포트: 3999)."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                # 임무 전송용 포트는 3999 사용
                s.connect((drone_ip, 3999))
                s.sendall(message.encode('utf-8'))
                print(f"드론({drone_ip})으로 순찰 경로 전송 완료.")
                # tkinter는 메인 스레드에서만 GUI 업데이트 가능
                self.root.after(0, lambda: tkinter.messagebox.showinfo("전송 완료", "드론으로 순찰 경로를 전송했습니다."))
        except Exception as e:
            print(f"드론으로 명령 전송 실패: {e}")
            self.root.after(0, lambda: tkinter.messagebox.showerror("전송 실패", f"드론에 연결할 수 없습니다.\nIP 주소와 네트워크를 확인해주세요.\n\n오류: {e}"))
            
    # --- 이하 함수들은 이전 버전과 거의 동일 (주석 정리) ---
    def add_waypoint_by_click(self, coords): self._add_waypoint_marker(coords)
    def add_waypoint_by_coord(self):
        try:
            lat, lon = float(self.lat_entry.get()), float(self.lon_entry.get())
            self._add_waypoint_marker((lat, lon)); self.lat_entry.delete(0, tkinter.END); self.lon_entry.delete(0, tkinter.END)
        except ValueError: tkinter.messagebox.showerror("입력 오류", "올바른 숫자 형식의 위도와 경도를 입력하세요.")
    def _add_waypoint_marker(self, coords):
        marker = self.map_widget.set_marker(coords[0], coords[1], text=f"WP {len(self.waypoints) + 1}")
        self.waypoints.append(coords); self.markers.append(marker)
    def delete_last_waypoint(self):
        if self.markers: self.markers.pop().delete(); self.waypoints.pop()
        else: tkinter.messagebox.showwarning("경고", "삭제할 경로점이 없습니다.")
    def clear_all_waypoints(self):
        for marker in self.markers: marker.delete()
        self.waypoints.clear(); self.markers.clear()
    def clear_fire_pins(self):
        if not self.fire_markers: tkinter.messagebox.showinfo("정보", "삭제할 화재 핑이 없습니다."); return
        for marker in self.fire_markers: marker.delete()
        self.fire_markers.clear()
    def start_alert_server(self):
        server_thread = threading.Thread(target=self.run_alert_server, daemon=True)
        server_thread.start()
    def run_alert_server(self):
        """화재 경보 수신 서버 로직 (포트: 4000)"""
        host = '0.0.0.0'
        port = 4000
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port)); s.listen()
            print(f"화재 경보 수신 대기 중... IP: {socket.gethostbyname(socket.gethostname())}, Port: {port}")
            while True:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=self.handle_alert_client, args=(conn, addr), daemon=True)
                client_thread.start()
    def handle_alert_client(self, conn, addr):
        """화재 경보 데이터를 수신하고 처리합니다."""
        print(f"{addr} 에서 화재 경보 연결됨.")
        with conn:
            data = conn.recv(1024)
            if data:
                try:
                    decoded_data = data.decode('utf-8')
                    lat, lon = map(float, decoded_data.split(','))
                    self.map_widget.set_position(lat, lon); self.map_widget.set_zoom(17)
                    fire_marker = self.map_widget.set_marker(lat, lon, text="!!화재 발생!!", marker_color_circle="red", marker_color_outside="red")
                    self.fire_markers.append(fire_marker)
                    tkinter.messagebox.showerror("긴급!", f"화재 발생!\n위치: {lat}, {lon}")
                except Exception as e: print(f"경보 데이터 처리 오류: {e}")
        print(f"{addr} 화재 경보 연결 종료.")

if __name__ == "__main__":
    root = tkinter.Tk()
    app = App(root)
    root.mainloop()
