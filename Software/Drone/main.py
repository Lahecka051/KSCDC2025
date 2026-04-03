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
    active_threads = []

    try:
        print("[메인] 화재 탐지 및 소화 시스템 시작")
        print("[메인] 화재 탐지: 활성화")
        print("[메인] 화재 진압: 활성화")

        # 드론 연결
        if not drone_system.connect():
            print("[메인] 드론 연결 실패")
            return

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

                        # [최적화] 완료된 스레드를 리스트에서 제거 (리스트 컴프리헨션으로 한 줄 처리)
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

                                communicator.send_status_update("EXTINGUISH_START", None)

                                mission_thread = threading.Thread(
                                    target=fire_extinguishing.run,
                                    args=(target,),
                                    daemon=True,
                                    name="ExtinguishMission"
                                )
                                mission_thread.start()
                                active_threads.append(mission_thread)
                            else:
                                print("[메인] 잘못된 화재 진압 목표 좌표")

                        # 순찰 명령 처리
                        elif isinstance(command_data, list):
                            if len(command_data) > 0:
                                print(f"[메인] 순찰 경로 수신: {len(command_data)}개 지점")

                                communicator.send_status_update("PATROL_START", None)

                                mission_thread = threading.Thread(
                                    target=patrol.run,
                                    args=(command_data,),
                                    daemon=True,
                                    name="PatrolMission"
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

                time.sleep(0.1)

            # [최적화] socket.timeout 제거 — receive_data()는 DroneCommunicator 내부에서
            # accept() 블로킹으로 동작하므로, 여기서 socket.timeout은 발생하지 않음.
            # 불필요한 예외 핸들러를 제거하여 코드 명확성 향상.
            except Exception as e:
                print(f"[메인] 수신 오류: {e}")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n[메인] 프로그램 종료 중...")

        print("[메인] 진행중인 미션 중단...")
        for thread in active_threads:
            if thread.is_alive():
                print(f"[메인] {thread.name} 종료 대기...")
                thread.join(timeout=2)

        if drone_system.is_armed:
            print("[메인] 긴급 착륙...")
            try:
                drone_system.set_mode_land()
                time.sleep(5)
                drone_system.disarm()
            # [최적화] bare except → except Exception
            # 기존: except: (SystemExit, KeyboardInterrupt 등 시스템 예외까지 무시)
            # 수정: except Exception (일반 예외만 포착, 시스템 예외는 전파)
            except Exception:
                print("[메인] 긴급 착륙 실패")

    except Exception as e:
        print(f"[메인] 치명적 오류: {e}")

    finally:
        # 리소스 정리
        print("[메인] 리소스 정리 중...")

        # [최적화] bare except → except Exception (3개소 동일 적용)
        # bare except는 KeyboardInterrupt, SystemExit 등도 삼키므로,
        # 종료 과정에서 Ctrl+C 재입력 시 프로세스가 멈출 수 있음.
        try:
            servo.cleanup()
        except Exception:
            pass

        try:
            cap0.release()
            cap1.release()
            cv2.destroyAllWindows()
        except Exception:
            pass

        print("[메인] 종료 완료")

if __name__ == '__main__':
    main()
