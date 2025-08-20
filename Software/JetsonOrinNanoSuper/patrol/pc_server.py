# PC (관제 센터)용 코드
import tkinter
import tkinter.messagebox
from tkintermapview import TkinterMapView
import socket
import threading
import json
from PIL import Image, ImageTk
import io
import os
import time

class App:
    def __init__(self, root_widget):
        self.root = root_widget
        self.root.title("화재 감시 드론 관제 센터")
        self.root.geometry("1400x650")

        # --- GUI 프레임 설정 ---
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=0)
        
        self.left_frame = tkinter.Frame(self.root)
        self.left_frame.grid(row=0, column=0, sticky="nsew")
        self.map_frame = tkinter.Frame(self.left_frame)
        self.map_frame.pack(fill="both", expand=True)
        
        self.control_frame = tkinter.Frame(self.root, pady=10)
        self.control_frame.grid(row=1, column=0, columnspan=1, sticky="nsew")
        
        self.right_frame = tkinter.LabelFrame(self.root, text="수신된 현장 사진", width=700, padx=10, pady=10)
        self.right_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        self.image_label = tkinter.Label(self.right_frame, text="사진 수신 대기 중...", bg="lightgray")
        self.image_label.pack(fill="both", expand=True)

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
        tkinter.Label(self.mission_run_frame, text="드론 주소소:").pack(side="left", padx=5)
        self.drone_ip_entry = tkinter.Entry(self.mission_run_frame)
        self.drone_ip_entry.pack(side="left", padx=5)
        self.start_button = tkinter.Button(self.mission_run_frame, text="순찰 시작", bg="lightblue", command=self.start_patrol)
        self.start_button.pack(side="left", padx=10, pady=10)

        # --- 지도 위젯 및 변수 초기화 ---
        self.map_widget = TkinterMapView(self.map_frame, width=600, height=500, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(37.5665, 126.9780)
        self.map_widget.set_zoom(15)
        self.map_widget.add_right_click_menu_command(label="여기에 경로점 추가", command=self.add_waypoint_by_click, pass_coords=True)
        self.waypoints = []
        self.markers = []
        self.fire_markers = []
        
        # 보고 수신 서버 시작 (포트: 4000)
        self.start_report_server()

    # --- 기능 수정: 화재 보고 처리 로직 ---
    def handle_fire_report(self, conn, addr):
        """클라이언트로부터 화재 보고(좌표, 사진)를 한번에 수신하고 처리합니다."""
        print(f"{addr} 에서 화재 보고 연결됨.")
        try:
            with conn:
                # 1. 헤더 수신 (좌표, 이미지 크기 등의 메타데이터)
                #    헤더는 1024바이트의 고정 크기로 받기로 약속합니다.
                header_data = conn.recv(1024)
                if not header_data:
                    print("헤더 수신 실패.")
                    return
                
                header = json.loads(header_data.decode('utf-8').strip())
                coordinates = header['coordinates']
                img_size = header['image_size']

                print(f"헤더 수신: 좌표={coordinates}, 이미지 크기={img_size} 바이트")

                # 2. 이미지 데이터 수신
                conn.sendall(b"HEADER_OK") # 헤더 잘 받았다는 신호 전송
                img_data = b''
                while len(img_data) < img_size:
                    chunk = conn.recv(4096)
                    if not chunk: break
                    img_data += chunk
                
                print(f"이미지 수신 완료: {len(img_data)} 바이트")

                # --- 3. GUI 동시 업데이트 ---
                # 메인 스레드에서 GUI를 업데이트하도록 작업을 예약합니다.
                self.root.after(0, self.update_gui_with_report, coordinates, img_data)

        except Exception as e:
            print(f"보고 처리 중 오류 발생: {e}")
        finally:
            print(f"{addr} 화재 보고 연결 종료.")
    
    def update_gui_with_report(self, coords, img_data):
        """수신된 좌표와 이미지 데이터로 GUI를 업데이트합니다."""
        # 지도 이동 및 핑 표시
        lat, lon = coords['lat'], coords['lon']
        self.map_widget.set_position(lat, lon)
        self.map_widget.set_zoom(17)
        fire_marker = self.map_widget.set_marker(lat, lon, text="!!화재 발생!!", marker_color_circle="red", marker_color_outside="red")
        self.fire_markers.append(fire_marker)
        tkinter.messagebox.showerror("긴급!", f"화재 발생!\n위치: {lat}, {lon}")

        # 이미지 표시 및 저장
        try:
            image = Image.open(io.BytesIO(img_data))
            save_dir = "captures"
            os.makedirs(save_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(save_dir, f"fire_{timestamp}.jpg")
            image.save(filepath)
            print(f"이미지가 다음 경로에 저장되었습니다: {filepath}")

            image.thumbnail((self.right_frame.winfo_width(), self.right_frame.winfo_height()))
            photo = ImageTk.PhotoImage(image)
            self.image_label.config(image=photo, text="")
            self.image_label.image = photo
        except Exception as e:
            print(f"이미지 표시/저장 오류: {e}")
            self.image_label.config(text="이미지 처리 오류")

    # --- 나머지 함수들 (이전과 거의 동일) ---
    def start_patrol(self):
        drone_ip = self.drone_ip_entry.get()
        if not drone_ip: tkinter.messagebox.showerror("오류", "드론의 IP 주소를 입력해주세요."); return
        if not self.waypoints: tkinter.messagebox.showerror("오류", "순찰을 시작하기 전에 경로를 먼저 설정해주세요."); return
        path_data = [{"lat": lat, "lon": lon} for lat, lon in self.waypoints]
        message = json.dumps(path_data)
        thread = threading.Thread(target=self.send_command_to_drone, args=(drone_ip, message), daemon=True)
        thread.start()
    def send_command_to_drone(self, drone_ip, message):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((drone_ip, 3999))
                s.sendall(message.encode('utf-8'))
                self.root.after(0, lambda: tkinter.messagebox.showinfo("전송 완료", "드론으로 순찰 경로를 전송했습니다."))
        except Exception as e:
            self.root.after(0, lambda: tkinter.messagebox.showerror("전송 실패", f"드론에 연결할 수 없습니다.\n오류: {e}"))
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
    def start_report_server(self):
        server_thread = threading.Thread(target=self.run_report_server, daemon=True)
        server_thread.start()
    def run_report_server(self):
        host = '0.0.0.0'; port = 4000
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port)); s.listen()
            
            pc_hostname = socket.gethostname()
            print(f"보고 수신 대기 중... PC 이름: {pc_hostname}.local, Port: {port}")
            while True:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=self.handle_fire_report, args=(conn, addr), daemon=True)
                client_thread.start()

if __name__ == "__main__":
    root = tkinter.Tk()
    app = App(root)
    root.mainloop()
