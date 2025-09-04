# Landing.py
import cv2
import numpy as np
import time

class Landing:
    def __init__(self, cap, drone_system, marker_path="/home/kscdc2025/Marker.png"):
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
        self.last_command = [0, 0, 0, 0]  # 마지막 명령 저장
        
        # 착륙 파라미터
        self.LANDING_SPEED = 0.2  # 하강 속도 (m/s)
        self.POSITION_THRESHOLD = 30  # 위치 오차 임계값 (픽셀)
        self.AREA_THRESHOLD = 5000  # 착륙 완료 판정 면적
        
        # 수정: 시간 기반 마커 탐색
        self.marker_search_start = None  # 마커 탐색 시작 시간
        self.MARKER_SEARCH_TIMEOUT = 10  # 10초 타임아웃

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
        kernel = np.ones((3,3), np.uint8)
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
        
        # 화면 중앙 좌표
        center_x = self.frame_width // 2
        center_y = self.frame_height // 2
        
        # 마커 탐지
        marker_info = self.detect_red_dot(frame)
        
        # 기본 명령 (호버링)
        command = [0, 0, 0, 0]
        
        if marker_info:
            marker_center_x, marker_center_y, area = marker_info
            
            # 수정: 마커 찾으면 타이머 리셋
            self.marker_search_start = None
            
            # 마커의 상대적 위치
            x_diff = marker_center_x - center_x
            y_diff = marker_center_y - center_y
            
            # 명령 계산을 위한 스케일링 팩터
            SCALE = 300.0  # 픽셀을 m/s로 변환
            
            # [전진/후진, 좌/우, 상승/하강, 방향]
            
            # 전진/후진 제어 - Y축 오차에 비례
            command[0] = (y_diff / SCALE)
            
            # 좌/우 제어 - X축 오차에 비례
            command[1] = -(x_diff / SCALE)
            
            # 하강 - 착륙 중이므로 항상 하강
            command[2] = -self.LANDING_SPEED
            
            # 회전 - 사용하지 않음
            command[3] = 0
            
            # 속도 제한
            command[0] = max(-0.5, min(0.5, command[0]))  # 전진/후진 제한
            command[1] = max(-0.5, min(0.5, command[1]))  # 좌/우 제한
            
            # 디버그 메시지 출력 (0.5초 간격)
            now = time.time()
            if now - self.last_print_time > 0.5:
                print(f"[착륙] 마커 감지 | 오차: x={x_diff:.0f}, y={y_diff:.0f} | 면적: {area:.0f} | 명령: [{command[0]:.2f}, {command[1]:.2f}, {command[2]:.2f}, {command[3]:.2f}]")
                self.last_print_time = now
                
            # 착륙 완료 판정 (마커가 충분히 크고 중앙에 위치)
            if area > self.AREA_THRESHOLD and abs(x_diff) < self.POSITION_THRESHOLD and abs(y_diff) < self.POSITION_THRESHOLD:
                return "LANDING_COMPLETE"
            
        else:
            # 수정: 마커 미감지 시 타이머 체크
            if self.marker_search_start is None:
                self.marker_search_start = time.time()
                print(f"[착륙] 마커 탐색 시작 (최대 {self.MARKER_SEARCH_TIMEOUT}초)")
            
            elapsed = time.time() - self.marker_search_start
            
            if elapsed >= self.MARKER_SEARCH_TIMEOUT:
                # 10초 타임아웃 - LAND 모드 전환
                print(f"[착륙] 마커 탐색 실패 ({self.MARKER_SEARCH_TIMEOUT}초 초과) - LAND 모드 전환")
                return "FORCE_LAND"
            else:
                # 호버링하며 대기
                remaining = self.MARKER_SEARCH_TIMEOUT - elapsed
                if int(elapsed) % 2 == 0:  # 2초마다 출력
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
        
        # 수정: goto_home이 이미 nonfire_patrol.py에서 호출되었으므로
        # 여기서는 홈 도착을 가정하고 바로 마커 탐색 시작
        print("[착륙] 홈 위치 도착 확인 - 마커 탐색 시작")
        
        # 착륙 모드 설정
        self.drone_system.set_mode_guided()  # GUIDED 모드 유지하여 정밀 제어
        
        landing_complete = False
        start_time = time.time()
        timeout = 60  # 전체 착륙 최대 60초
        
        try:
            while not landing_complete and (time.time() - start_time) < timeout:
                ret, frame = self.cap.read()
                if not ret:
                    print("[착륙] 카메라 프레임 읽기 실패")
                    time.sleep(0.1)
                    continue
                
                # 제어 명령 얻기
                command = self.get_control_command(frame)
                
                # 수정: 강제 LAND 모드 체크
                if command == "FORCE_LAND":
                    print("[착륙] LAND 모드로 전환")
                    self.drone_system.set_mode_land()
                    time.sleep(10)  # 착륙 대기
                    landing_complete = True
                    break
                
                # 착륙 완료 체크
                elif command == "LANDING_COMPLETE":
                    print("[착륙] 착륙 완료 감지! 시동 끄기...")
                    landing_complete = True
                    break
                
                # 드론에 명령 전송
                else:
                    self.drone_system.set_command(command)
                
                # 짧은 대기 (제어 주기)
                time.sleep(0.05)  # 20Hz 제어
                
        except KeyboardInterrupt:
            print("[착륙] 사용자 중단")
            
        except Exception as e:
            print(f"[착륙] 오류 발생: {e}")
            
        finally:
            # 착륙 완료 또는 타임아웃 시 처리
            if landing_complete:
                print("[착륙] 정상 착륙 완료")
            else:
                print("[착륙] 착륙 타임아웃 - 강제 착륙 모드 전환")
                self.drone_system.set_mode_land()
                time.sleep(10)
            
            # 시동 끄기
            self.drone_system.disarm()
            print("[착륙] 시동 꺼짐. 착륙 완료!")
            
        return landing_complete
