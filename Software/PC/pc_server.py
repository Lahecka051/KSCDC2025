# PC (관제 센터)용 최종 코드
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
import queue
from pc_rpi_server import PC_RPi_Server

class App:
    def __init__(self, root_widget):
        """
        클래스의 생성자 함수입니다.
        GUI의 모든 위젯(버튼, 프레임, 지도 등)을 생성하고 초기화하며,
        백그라운드에서 드론의 보고를 받을 서버 스레드를 시작합니다.
        """
        self.root = root_widget
        self.root.title("화재 감시 드론 관제 센터")
        self.root.geometry("1400x650")

        # --- GUI 레이아웃 프레임 설정 ---
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=0)

        # 좌측 프레임 (지도)
        self.left_frame = tkinter.Frame(self.root)
        self.left_frame.grid(row=0, column=0, sticky="nsew")
        self.map_frame = tkinter.Frame(self.left_frame)
        self.map_frame.pack(fill="both", expand=True)

        # 하단 제어 프레임 (모든 컨트롤 버튼을 담는 영역)
        self.control_frame = tkinter.Frame(self.root, pady=10)
        self.control_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

        # 우측 프레임 (수신된 사진)
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
        tkinter.Label(self.mission_run_frame, text="드론 주소:").pack(side="left", padx=5)
        self.drone_ip_entry = tkinter.Entry(self.mission_run_frame)
        self.drone_ip_entry.pack(side="left", padx=5)
        self.start_button = tkinter.Button(self.mission_run_frame, text="순찰 시작", bg="lightblue", command=self.start_patrol)
        self.start_button.pack(side="left", padx=10, pady=10)

        # 4. 화재 대응 프레임
        self.response_frame = tkinter.LabelFrame(self.control_frame, text="화재 대응")
        self.response_frame.pack(side="left", padx=10, pady=5, fill="y")

        self.approve_button = tkinter.Button(self.response_frame, text="화재진압 승인", bg="orangered", state="disabled", command=self.approve_extinguish_mission)
        self.approve_button.pack(side="left", padx=10, pady=10)

        self.mission_status_label = tkinter.Label(self.response_frame, text="상태: 대기 중")
        self.mission_status_label.pack(side="left", padx=10)

        # --- 지도 위젯 및 변수 초기화 ---
        self.map_widget = TkinterMapView(self.map_frame, width=600, height=500, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(35.1422, 129.0968) # 초기 위치: 경성대 공대
        self.map_widget.set_zoom(15)
        self.map_widget.add_right_click_menu_command(label="여기에 경로점 추가", command=self.add_waypoint_by_click, pass_coords=True)

        # 미션 관련 데이터 저장 변수
        self.waypoints = []
        self.markers = []
        self.fire_markers = []
        self.last_fire_location = None

        # 라즈베리 파이 서버 관련 변수
        self.rpi = PC_RPi_Server()

        # 보고 수신 서버 시작
        self.start_drone_report_server()
        self.rpi.start()

        # 소화볼 관련 변수
        self.ball_q = queue.Queue()
        self.ball = 0
        self.complete = queue.Queue()

    def handle_drone_report(self, conn, addr):
        """
        (네트워크 스레드) 드론으로부터 오는 모든 보고(화재, 상태 등)를 처리합니다.
        메시지 유형('type')을 확인하여 화재 보고와 상태 보고를 구분합니다.
        """
        print(f"드론({addr})으로부터 통신 연결됨.")
        try:
            with conn:
                raw_data = conn.recv(1024)
                if not raw_data: return

                report_data = json.loads(raw_data.decode('utf-8').strip())
                report_type = report_data.get('type')

                if report_type == 'FIRE_REPORT':
                    print("수신 메시지 유형: 화재 보고")
                    coordinates = report_data['coordinates']
                    img_size = report_data['image_size']
                    print(f"헤더 수신: 좌표={coordinates}, 이미지 크기={img_size} 바이트")

                    conn.sendall(b"HEADER_OK")

                    # [최적화] 이미지 수신 루프에 타임아웃 추가
                    # 기존: 타임아웃 없음 — 드론이 이미지 전송 중 연결 끊기면 무한 대기
                    # 수정: 10초 타임아웃 설정
                    conn.settimeout(10)
                    img_data = b''
                    try:
                        while len(img_data) < img_size:
                            chunk = conn.recv(4096)
                            if not chunk: break
                            img_data += chunk
                    except socket.timeout:
                        print(f"[경고] 이미지 수신 타임아웃 ({len(img_data)}/{img_size} 바이트)")

                    print(f"이미지 수신 완료: {len(img_data)} 바이트")
                    self.root.after(0, self.update_gui_with_report, coordinates, img_data)

                elif report_type == 'STATUS_UPDATE':
                    status = report_data.get('status')
                    ball_count = report_data.get('ball count')
                    print(f"수신 메시지 유형: 상태 보고, 내용: {status}")
                    if status == 'EXTINGUISH_COMPLETE':
                        self.root.after(0, self.mission_status_label.config, {"text": "상태: 진압 완료. 복귀 확인."})
                        self.ball_q.put(ball_count)

        except Exception as e:
            print(f"드론 보고 처리 중 오류 발생: {e}")
        finally:
            print(f"드론({addr}) 통신 연결 종료.")

    # [최적화] 중복 메서드 정의 제거
    # 기존: update_gui_with_report가 163~167행과 167~213행에 2번 정의되어 있었음
    #       Python에서 같은 이름의 메서드를 재정의하면 첫 번째 정의가 무시됨
    #       첫 번째 정의(빈 메서드)가 두 번째를 덮어쓸 일은 없지만,
    #       코드 혼란을 유발하는 명백한 복사-붙여넣기 실수
    # 수정: 첫 번째 빈 정의 제거, 실제 구현만 유지
    def update_gui_with_report(self, coords, img_data):
        """
        (메인 GUI 스레드) 수신된 화재 보고 데이터로 지도, 이미지, 버튼 등 GUI를 업데이트합니다.
        """
        if coords is not None:
            lat, lon = coords['lat'], coords['lon']
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(17)
            fire_marker = self.map_widget.set_marker(lat, lon, text="!!화재 발생!!", marker_color_circle="red", marker_color_outside="red")
            self.fire_markers.append(fire_marker)
            tkinter.messagebox.showerror("긴급!", f"화재 발생!\n위치: {lat}, {lon}")

            self.last_fire_location = coords
            self.approve_button.config(state="normal")
            self.mission_status_label.config(text="상태: 진압 승인 대기 중")
        else:
             tkinter.messagebox.showerror("오류!", "드론이 GPS 좌표 오류")

        try:
            image = Image.open(io.BytesIO(img_data))

            script_dir = os.path.dirname(os.path.abspath(__file__))
            save_dir = os.path.join(script_dir, "captures")
            os.makedirs(save_dir, exist_ok=True)

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(save_dir, f"fire_{timestamp}.jpg")
            image.save(filepath)
            print(f"이미지가 다음 경로에 저장되었습니다: {filepath}")

            image.thumbnail((640, 480))

            photo = ImageTk.PhotoImage(image)
            self.image_label.config(image=photo, text="")
            self.image_label.image = photo

        except Exception as e:
            error_message = f"이미지 표시/저장 오류: {e}"
            print(error_message)
            self.image_label.config(text="이미지 처리 오류")
            tkinter.messagebox.showerror("GUI 업데이트 오류", error_message)

    def start_patrol(self):
        """'순찰 시작' 버튼 클릭 시, 설정된 경로점들을 드론에 전송하도록 명령합니다."""
        drone_address = self.drone_ip_entry.get()
        if not drone_address:
            tkinter.messagebox.showerror("오류", "드론의 주소(이름 또는 IP)를 입력해주세요.")
            return
        if not self.waypoints:
            tkinter.messagebox.showerror("오류", "순찰을 시작하기 전에 경로를 먼저 설정해주세요.")
            return

        path_data = [{"lat": lat, "lon": lon} for lat, lon in self.waypoints]
        message = json.dumps(path_data)
        thread = threading.Thread(target=self.send_command_to_drone, args=(drone_address, message), daemon=True)
        thread.start()

    def send_command_to_drone(self, drone_address, message):
        """
        (네트워크 스레드) 드론에 실제 명령(순찰, 진압) 메시지를 TCP 소켓으로 전송합니다.
        """
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                # [최적화] 연결 타임아웃 추가
                # 기존: 타임아웃 없음 — 드론 주소가 잘못되면 OS 기본 타임아웃(~2분)까지 GUI 블로킹
                # 수정: 5초 타임아웃으로 빠른 실패 피드백 제공
                s.settimeout(5)
                s.connect((drone_address, 65523))
                s.sendall(message.encode('utf-8'))
                try:
                    msg_data = json.loads(message)
                    if isinstance(msg_data, list):
                         self.root.after(0, lambda: tkinter.messagebox.showinfo("전송 완료", "드론으로 순찰 경로를 전송했습니다."))
                except Exception:
                    pass
        except Exception as e:
            self.root.after(0, lambda: tkinter.messagebox.showerror("전송 실패", f"드론에 연결할 수 없습니다.\n오류: {e}"))

    def approve_extinguish_mission(self):
        """'화재진압 승인' 버튼 클릭 시, 소화볼 장전 및 진압 임무 시작 스레드를 실행합니다."""
        if self.last_fire_location is None:
            tkinter.messagebox.showwarning("경고", "보고된 화재 위치가 없습니다.")
            return

        self.approve_button.config(state="disabled")
        self.mission_status_label.config(text="상태: 소화볼 장전 중...")
        thread = threading.Thread(target=self._extinguish_mission_thread, daemon=True)
        thread.start()

    def _extinguish_mission_thread(self):
        """
        (백그라운드 스레드) 소화볼 장전을 시뮬레이션하고 드론에 진압 명령을 전송합니다.
        """
        try:
            self.ball = self.ball_q.get_nowait()
        except queue.Empty:
            pass

        cmd = [f"{self.ball}"]
        self.rpi.send_command(cmd)
        print("\n[관제 센터] 스테이션에 소화볼 장전 명령 전송...")
        tmp = self.rpi.response_q.get()
        if tmp:
            print("[관제 센터] 소화볼 장전 완료.")
            print("[STATION] :", tmp)
            # [최적화] 매직 넘버를 의미 있는 상수로 표현
            # 기존: self.ball = 2 (의미 불명확)
            # 수정: 주석 추가로 의미 명확화 — 장전 완료 후 소화볼 2발 장착 상태
            self.ball = 2  # 소화볼 장전 완료 수량

            self.root.after(0, self.mission_status_label.config, {"text": "상태: 드론 출동 명령 전송 중..."})

            drone_address = self.drone_ip_entry.get()
            if not drone_address:
                self.root.after(0, tkinter.messagebox.showerror, ("오류", "드론 주소를 찾을 수 없습니다."))
                self.root.after(0, self.mission_status_label.config, {"text": "오류: 드론 주소 없음"})
                return

            command = {
                "type": "EXTINGUISH",
                "target": self.last_fire_location
            }
            message = json.dumps(command)

            self.send_command_to_drone(drone_address, message)
            self.root.after(0, self.mission_status_label.config, {"text": "상태: 진압 임무 출동 완료!"})
        else:
            print("[관제 센터] 스테이션 응답 오류 - 장전 실패")

    def add_waypoint_by_click(self, coords):
        """지도에서 마우스 우클릭 시 호출되어 해당 위치에 경로점을 추가합니다."""
        self._add_waypoint_marker(coords)

    def add_waypoint_by_coord(self):
        """'좌표로 추가' 버튼 클릭 시, 입력된 위도/경도 값으로 경로점을 추가합니다."""
        try:
            lat, lon = float(self.lat_entry.get()), float(self.lon_entry.get())
            self._add_waypoint_marker((lat, lon))
            self.lat_entry.delete(0, tkinter.END)
            self.lon_entry.delete(0, tkinter.END)
        except ValueError:
            tkinter.messagebox.showerror("입력 오류", "올바른 숫자 형식의 위도와 경도를 입력하세요.")

    def _add_waypoint_marker(self, coords):
        """(내부 함수) 지도에 마커를 그리고 경로점 리스트에 좌표를 추가하는 공통 로직입니다."""
        marker = self.map_widget.set_marker(coords[0], coords[1], text=f"WP {len(self.waypoints) + 1}")
        self.waypoints.append(coords)
        self.markers.append(marker)

    def delete_last_waypoint(self):
        """마지막으로 추가된 경로점과 마커를 삭제합니다."""
        if self.markers:
            self.markers.pop().delete()
            self.waypoints.pop()
        else:
            tkinter.messagebox.showwarning("경고", "삭제할 경로점이 없습니다.")

    def clear_all_waypoints(self):
        """설정된 모든 경로점과 마커를 초기화합니다."""
        for marker in self.markers:
            marker.delete()
        self.waypoints.clear()
        self.markers.clear()

    def clear_fire_pins(self):
        """지도에 표시된 모든 화재 발생 지점 핑(마커)을 삭제합니다."""
        if not self.fire_markers:
            tkinter.messagebox.showinfo("정보", "삭제할 화재 핑이 없습니다.")
            return
        for marker in self.fire_markers:
            marker.delete()
        self.fire_markers.clear()

    def start_drone_report_server(self):
        """드론의 보고를 수신할 서버를 별도의 스레드에서 시작합니다."""
        drone_server_thread = threading.Thread(target=self.run_drone_report_server, daemon=True)
        drone_server_thread.start()

    def run_drone_report_server(self):
        """(네트워크 스레드) TCP 소켓 서버를 열고 드론의 연결을 무한 대기합니다."""
        host = '0.0.0.0'
        port = 65524
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()

            pc_hostname = socket.gethostname()
            print(f"드론 보고 수신 대기 중... PC 이름: {pc_hostname}.local, Port: {port}")

            while True:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=self.handle_drone_report, args=(conn, addr), daemon=True)
                client_thread.start()


# 이 스크립트 파일이 직접 실행될 때만 아래 코드를 실행
if __name__ == "__main__":
    root = tkinter.Tk()
    app = App(root)
    root.mainloop()
