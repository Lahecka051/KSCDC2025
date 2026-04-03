# Landing.py
import cv2
import numpy as np
import time

class Landing:
    def __init__(self, cap, drone_system, marker_path="/home/kscdc2025/drone/Marker.png"):
        """
        착륙 시스템 초기화
        :param cap: 하단 카메라 (cap1)
        :param drone_system: DroneController 인스턴스
        :param marker_path: 마커 이미지 경로 (현재 미사용, 추후 확장용)
        """
        self.cap = cap
        self.drone_system = drone_system
        self.marker_path = marker_path

        # 프레임 크기 (cap에서 가져옴)
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.last_print_time = 0
        self.last_command = [0, 0, 0, 0]

        # 착륙 파라미터
        self.LANDING_SPEED = 0.2
        self.POSITION_THRESHOLD = 30
        self.AREA_THRESHOLD = 5000

        # 시간 기반 마커 탐색
        self.marker_search_start = None
        self.MARKER_SEARCH_TIMEOUT = 10

        # [최적화] 고도 체크 주기 상수 추가
        # 기존: 매 프레임(20Hz)마다 read_altitude() 호출 → recv_match(timeout=1) 블로킹으로
        #       실제 제어 주기가 1Hz 이하로 저하 (20Hz 루프가 사실상 무의미)
        # 수정: N 프레임마다 1회 고도 체크하여 제어 루프 성능 유지
        self.ALT_CHECK_INTERVAL = 20  # 20프레임(약 1초)마다 고도 체크

    def detect_red_dot(self, frame):
        """프레임에서 가장 큰 빨간 점을 감지하고 중심 좌표와 면적을 반환"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 정의
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 150, 150])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            approx = cv2.approxPolyDP(largest_contour, 0.02 * cv2.arcLength(largest_contour, True), True)

            if area > 150 and len(approx) > 7:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy, area)
        return None

    def get_control_command(self, frame):
        """
        카메라 프레임을 분석하여 드론 제어 명령을 반환.
        DroneController 형식: [전진/후진, 좌/우, 상승/하강, 방향]
        """

        center_x = self.frame_width // 2
        center_y = self.frame_height // 2

        marker_info = self.detect_red_dot(frame)

        command = [0, 0, 0, 0]

        if marker_info:
            marker_center_x, marker_center_y, area = marker_info

            # 마커 찾으면 타이머 리셋
            self.marker_search_start = None

            x_diff = marker_center_x - center_x
            y_diff = marker_center_y - center_y

            SCALE = 300.0

            command[0] = (y_diff / SCALE)
            command[1] = -(x_diff / SCALE)
            command[2] = -self.LANDING_SPEED
            command[3] = 0

            command[0] = max(-0.5, min(0.5, command[0]))
            command[1] = max(-0.5, min(0.5, command[1]))

            now = time.time()
            if now - self.last_print_time > 0.5:
                print(f"[착륙] 마커 감지 | 오차: x={x_diff:.0f}, y={y_diff:.0f} | 면적: {area:.0f} | 명령: [{command[0]:.2f}, {command[1]:.2f}, {command[2]:.2f}, {command[3]:.2f}]")
                self.last_print_time = now

            if area > self.AREA_THRESHOLD and abs(x_diff) < self.POSITION_THRESHOLD and abs(y_diff) < self.POSITION_THRESHOLD:
                return "LANDING_COMPLETE"

        else:
            if self.marker_search_start is None:
                self.marker_search_start = time.time()
                print(f"[착륙] 마커 탐색 시작 (최대 {self.MARKER_SEARCH_TIMEOUT}초)")

            elapsed = time.time() - self.marker_search_start

            if elapsed >= self.MARKER_SEARCH_TIMEOUT:
                print(f"[착륙] 마커 탐색 실패 ({self.MARKER_SEARCH_TIMEOUT}초 초과) - LAND 모드 전환")
                return "FORCE_LAND"
            else:
                remaining = self.MARKER_SEARCH_TIMEOUT - elapsed
                if int(elapsed) % 2 == 0:
                    print(f"[착륙] 마커 탐색 중... (남은 시간: {remaining:.1f}초)")

        self.last_command = command
        return command

    def run(self):
        """
        착륙 시퀀스 실행
        - 홈 도착 후 마커 탐색 시작
        - 마커 추적하며 착륙
        - 착륙 완료 시 시동 끄기
        """
        print("\n[착륙] 착륙 시퀀스 시작")

        self.marker_search_start = None

        self.drone_system.set_mode_guided()

        landing_complete = False
        start_time = time.time()
        timeout = 30
        frame_error_count = 0
        max_frame_errors = 10
        # [최적화] 고도 체크 프레임 카운터 초기화
        frame_count = 0

        try:
            while not landing_complete and (time.time() - start_time) < timeout:
                if not self.cap.isOpened():
                    print("[착륙] 카메라 연결 끊김 - LAND 모드 전환")
                    self.drone_system.set_mode_land()
                    time.sleep(10)
                    break

                ret, frame = self.cap.read()
                if not ret:
                    frame_error_count += 1
                    print(f"[착륙] 카메라 프레임 읽기 실패 ({frame_error_count}/{max_frame_errors})")

                    if frame_error_count >= max_frame_errors:
                        print("[착륙] 카메라 오류 지속 - LAND 모드 전환")
                        self.drone_system.set_mode_land()
                        time.sleep(10)
                        break

                    time.sleep(0.1)
                    continue

                frame_error_count = 0

                command = self.get_control_command(frame)

                if command == "FORCE_LAND":
                    print("[착륙] LAND 모드로 전환")
                    self.drone_system.set_mode_land()
                    time.sleep(10)
                    landing_complete = True
                    break

                elif command == "LANDING_COMPLETE":
                    print("[착륙] 착륙 완료 감지! 시동 끄기...")
                    landing_complete = True
                    break

                else:
                    if not self.drone_system.set_command(command):
                        print("[착륙] 명령 전송 실패")

                time.sleep(0.05)  # 20Hz 제어

                # [최적화] 고도 체크 주기 분리
                # 기존: 매 프레임마다 read_altitude() 호출 (recv_match timeout=1초)
                #       → 제어 루프 50ms + 고도 읽기 1000ms = 실제 주기 ~1050ms (약 1Hz)
                # 수정: ALT_CHECK_INTERVAL 프레임(20회 = 약 1초)마다 1회 체크
                #       → 제어 루프가 실제로 20Hz로 동작, 고도는 비동기적으로 확인
                frame_count += 1
                if frame_count % self.ALT_CHECK_INTERVAL == 0:
                    current_alt = self.drone_system.read_altitude()
                    if current_alt is not None and current_alt < 0.3:
                        print(f"[착륙] 저고도 감지 ({current_alt:.2f}m) - 착륙 임박")

        except KeyboardInterrupt:
            print("[착륙] 사용자 중단")

        except Exception as e:
            print(f"[착륙] 오류 발생: {e}")
            try:
                self.drone_system.set_mode_land()
                time.sleep(10)
            # [최적화] bare except → except Exception
            except Exception:
                print("[착륙] 비상 착륙 명령 실패")

        finally:
            if landing_complete:
                print("[착륙] 정상 착륙 완료")
            else:
                print("[착륙] 착륙 타임아웃 - 강제 착륙 모드 전환")
                try:
                    self.drone_system.set_mode_land()
                    time.sleep(10)
                # [최적화] bare except → except Exception
                except Exception:
                    print("[착륙] 강제 착륙 명령 실패")

            try:
                self.drone_system.disarm()
                print("[착륙] 시동 꺼짐. 착륙 완료!")
            # [최적화] bare except → except Exception
            except Exception:
                print("[착륙] 시동 끄기 실패")

        return landing_complete
