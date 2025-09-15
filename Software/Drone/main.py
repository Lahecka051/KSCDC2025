# Jetson Orin Nano (드론)용 최종 코드
import socket
import threading
import time
import json
import os
import cv2

# 사용자가 만든 모듈
from DroneCommunicator import DroneCommunicator
from patrol import Patrol
from Fire_detector import Fire_detector
from drone_controller import DroneController
from Fire_extinguishing import Fire_extinguishing
from pipeline import gstreamer_pipeline
from Landing import Landing
from servo_control import Servo

# --- 드론 시스템 객체 생성 ---
drone_system = DroneController()

# --- 서보 객체 생성 ---
servo = Servo(servo_pin=33)

# --- 통신 모듈 객체 생성 ---
CONTROLLER_PC_HOSTNAME = 'AERO_17.local' 
communicator = DroneCommunicator(pc_hostname=CONTROLLER_PC_HOSTNAME)

# --- 카메라 객체 생성 ----
pipeline0 = gstreamer_pipeline(sensor_id=0)
pipeline1 = gstreamer_pipeline(sensor_id=1)
cap0 = cv2.VideoCapture(pipeline0, cv2.CAP_GSTREAMER)
cap1 = cv2.VideoCapture(pipeline1, cv2.CAP_GSTREAMER)

# --- 화재탐지 객체 생성 ---
fire_detector = Fire_detector(cap0, cap1)

# --- 착륙 시스템 객체 생성 ---
landing = Landing(cap1, drone_system)

# --- 순찰 객체 생성 ---
patrol = Patrol(drone_system, fire_detector, landing, communicator)

# --- 소화볼 투하 객체 생성 ---
fire_extinguishing = Fire_extinguishing(drone_system, fire_detector, landing, communicator, servo)

# --- 메인 로직: 화재 탐지 및 소화 시스템 ---
def main():
    # #수정: 전역 스레드 리스트 추가
    active_threads = []
    mission_in_progress = False  # #수정: 미션 진행 플래그
    
    try:
        print("[메인] 화재 탐지 및 소화 시스템 시작")
        print("[메인] 화재 탐지: 활성화")
        print("[메인] 화재 진압: 활성화")
        
        # 드론 연결
        if not drone_system.connect():
            print("[메인] 드론 연결 실패")
            return
        
        # #수정: 카메라 상태 확인
        if not cap0.isOpened() or not cap1.isOpened():
            print("[메인] 카메라 초기화 실패")
            return
            
        communicator.start_receiver()
        print("[메인] 명령 수신 대기 중...")
        
        while True:
            try:
                data = communicator.receive_data()
                if data:
                    try:
                        command_data = json.loads(data.decode('utf-8').strip())
                        
                        # #수정: 진행중인 미션 체크
                        active_threads = [t for t in active_threads if t.is_alive()]
                        if len(active_threads) > 0:
                            print(f"[메인] 현재 {len(active_threads)}개 미션 진행 중 - 새 명령 거부")
                            communicator.send_status_update("MISSION_IN_PROGRESS", None)
                            continue
                        
                        # 화재 진압 명령 처리
                        if isinstance(command_data, dict) and command_data.get('type') == 'EXTINGUISH':
                            target = command_data.get('target')
                            if target and 'lat' in target and 'lon' in target:
                                print(f"[메인] 화재 진압 명령 수신: 위도 {target['lat']:.7f}, 경도 {target['lon']:.7f}")
                                
                                # #수정: 미션 시작 알림
                                communicator.send_status_update("EXTINGUISH_START", None)
                                
                                mission_thread = threading.Thread(
                                    target=fire_extinguishing.run, 
                                    args=(target,), 
                                    daemon=True,
                                    name="ExtinguishMission"  # #수정: 스레드 이름 지정
                                )
                                mission_thread.start()
                                active_threads.append(mission_thread)
                            else:
                                print("[메인] 잘못된 화재 진압 목표 좌표")
                                
                        # 순찰 명령 처리
                        elif isinstance(command_data, list):
                            if len(command_data) > 0:
                                print(f"[메인] 순찰 경로 수신: {len(command_data)}개 지점")
                                
                                # #수정: 미션 시작 알림
                                communicator.send_status_update("PATROL_START", None)
                                
                                mission_thread = threading.Thread(
                                    target=patrol.run, 
                                    args=(command_data,), 
                                    daemon=True,
                                    name="PatrolMission"  # #수정: 스레드 이름 지정
                                )
                                mission_thread.start()
                                active_threads.append(mission_thread)
                            else:
                                print("[메인] 빈 순찰 경로")
                                
                        else:
                            print(f"[메인] 알 수 없는 명령: {command_data}")
                            
                    except json.JSONDecodeError as e:
                        print(f"[메인] JSON 파싱 오류: {e}")
                    except Exception as e:
                        print(f"[메인] 명령 처리 오류: {e}")
                        
                # #수정: 주기적으로 스레드 상태 체크
                time.sleep(0.1)
                
            except socket.timeout:
                # 타임아웃은 정상 동작
                pass
            except Exception as e:
                print(f"[메인] 수신 오류: {e}")
                time.sleep(1)
                    
    except KeyboardInterrupt:
        print("\n[메인] 프로그램 종료 중...")
        
        # #수정: 모든 스레드 종료 대기
        print("[메인] 진행중인 미션 중단...")
        for thread in active_threads:
            if thread.is_alive():
                print(f"[메인] {thread.name} 종료 대기...")
                thread.join(timeout=2)
        
        # #수정: 드론 안전 착륙
        if drone_system.is_armed:
            print("[메인] 긴급 착륙...")
            try:
                drone_system.set_mode_land()
                time.sleep(5)
                drone_system.disarm()
            except:
                print("[메인] 긴급 착륙 실패")
        
    except Exception as e:
        print(f"[메인] 치명적 오류: {e}")
        
    finally:
        # 리소스 정리
        print("[메인] 리소스 정리 중...")
        
        # #수정: 안전한 리소스 해제
        try:
            servo.cleanup()
        except:
            pass
            
        try:
            cap0.release()
            cap1.release()
            cv2.destroyAllWindows()
        except:
            pass
            
        print("[메인] 종료 완료")

if __name__ == '__main__':
    main()