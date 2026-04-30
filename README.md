# KSCDC2025 — Team Prometheus

<p align="center">
  <img alt="Status" src="https://img.shields.io/badge/Status-Final%20Submitted-success">
  <img alt="Contest" src="https://img.shields.io/badge/Contest-KSCDC%202025-blue">
  <img alt="Python" src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white">
  <img alt="Jetson" src="https://img.shields.io/badge/NVIDIA-Jetson%20Orin%20Nano%20Super-76B900?logo=nvidia&logoColor=white">
  <img alt="ArduPilot" src="https://img.shields.io/badge/ArduPilot-5.4.2-FF6F00">
  <img alt="YOLOv8" src="https://img.shields.io/badge/YOLOv8s-TensorRT-00FFFF">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-green">
</p>

> **스마트 드론을 활용한 실시간 화재 탐지 및 초기 소화 시스템**
>
> NVIDIA Jetson Orin Nano Super 기반, 딥러닝 객체인식을 통한 자율 순찰 및 소화볼 투하가 가능한 화재 대응 드론 시스템

---

## 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [팀원](#팀원)
3. [시스템 아키텍처](#시스템-아키텍처)
4. [운용 시나리오](#운용-시나리오)
5. [폴더 구조](#폴더-구조)
6. [소프트웨어 모듈 상세](#소프트웨어-모듈-상세)
7. [하드웨어 사양](#하드웨어-사양)
8. [소프트웨어 스택](#소프트웨어-스택)
9. [통신 프로토콜](#통신-프로토콜)
10. [대회 일정](#대회-일정)
11. [라이선스](#라이선스)

---

## 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 대회 | 2025 KSCDC (한국기계학회 창의설계경진대회) |
| 소속 | 경성대학교 |
| 기간 | 2025.05.02 ~ 2025.09.12 |
| 대회 홈페이지 | http://kscdc.ksme.or.kr/ |

## 팀원

| 이름 | 역할 |
|------|------|
| 이강민 | 조장 |
| 권혁준 | 팀원 |
| 송승진 | 팀원 |
| 유찬영 | 팀원 |
| 김철우 | 팀원 |

## 시스템 아키텍처

```
┌─────────────────┐       TCP/IP        ┌──────────────────┐       TCP/IP       ┌─────────────────┐
│   PC 관제 센터   │ ◄──────────────────► │   드론 (Jetson)   │                   │  Station (RPi)  │
│   (pc_server)   │    명령/보고 전송     │   (main.py)      │                   │  (station.py)   │
│                 │                      │                  │                   │                 │
│  - 지도 GUI     │  ← 화재 보고/이미지   │  - 화재 탐지      │                   │  - 소화볼 장전   │
│  - 경로 설정    │  → 순찰/진압 명령     │  - 자율 순찰      │                   │  - 도킹 정렬    │
│  - 진압 승인    │                      │  - 소화볼 투하    │                   │  - ToF 센서     │
│  - 사진 수신    │                      │  - 자동 착륙      │                   │                 │
└─────────────────┘                      └──────────────────┘                   └─────────────────┘
       :65524 (수신)                        :65523 (수신)                           :65525 (수신)
```

## 운용 시나리오

1. **순찰 명령** — PC 관제 센터에서 지도 위 경로점(Waypoint) 설정 후 드론에 전송
2. **자율 순찰** — 드론이 경로점을 따라 비행하며 정면 카메라(IMX477)로 YOLOv8s 화재/연기 탐지
3. **화재 확인** — 5초간 30프레임 이상 연속 감지 시 화재 확정, GPS 좌표 추정 후 화재 지점 이동
4. **하단 정렬** — 하단 카메라(IMX219)로 화재 위치에 드론 정밀 정렬
5. **화재 보고** — 현장 사진 촬영 후 PC 관제 센터로 좌표 및 이미지 전송
6. **홈 복귀 및 착륙** — 홈 좌표로 복귀 후, 빨간 마커 추적 기반 정밀 착륙
7. **진압 승인** — 관제 센터에서 화재 보고 확인 후 진압 승인 버튼 클릭
8. **소화볼 장전** — Station(Raspberry Pi)에 소화볼 장전 명령 전송
9. **소화볼 투하** — 드론이 화재 지점으로 재출격, 정렬 후 서보 모터로 소화볼 투하
10. **임무 완료** — 홈 복귀 및 착륙, 완료 상태 보고

## 폴더 구조

```
KSCDC2025/
├── README.md                          # 프로젝트 개요 (본 문서)
├── LICENSE                            # MIT License
├── .gitignore                         # Python/모델/로그 제외
├── .gitattributes                     # 줄바꿈 정규화 (LF)
│
├── Datasheets/                        # 부품 데이터시트 및 사양
│   ├── Filament.md                    #   3D 프린트 필라멘트
│   ├── Frame.md                       #   X6-X600 드론 프레임 (600mm)
│   ├── IMX219 Camera Module.md        #   하단 카메라 (160도 광각)
│   ├── IMX477 Camera Module.md        #   정면 카메라 (63도)
│   ├── JetsonOrinNanoSuper.md         #   NVIDIA Jetson Orin Nano Super
│   ├── MicoAir H743v2.md              #   FC (비행 컨트롤러)
│   ├── Motor & Propeller.md           #   Airgear 350 모터 & 프로펠러
│   └── Servo.md                       #   소화볼 투하용 서보 모터
│
├── Regular_Meeting/                   # 정기 회의록 (2025.05 ~ 2025.09)
│
├── Report/                            # 공식 보고서 (PDF)
│   ├── 설계제안서_프로메테우스_이강민.pdf
│   ├── 중간보고서_프로메테우스_이강민.pdf
│   └── 최종보고서_프로메테우스_이강민.pdf
│
├── STL/                               # 3D 프린트 모델 파일
│   ├── Drone/                         #   드론 탑재 부품 (소화볼 하우징/플레이트)
│   │   ├── Drone_4inch_Ball.stl
│   │   ├── Drone_BAT_House_Hook.stl
│   │   ├── Drone_BAT_House_Lower.stl
│   │   ├── Drone_BAT_House_Upper.stl
│   │   └── Drone_Bomb_Plate.stl
│   └── Station/                       #   스테이션 메커니즘 (랙앤피니언)
│       ├── Station_Bar.stl
│       ├── Station_Pinion.stl
│       └── Station_Rack.stl
│
└── Software/
    ├── Drone/                         # 드론 탑재 소프트웨어 (Jetson Orin Nano)
    │   ├── main.py                    #   메인 엔트리포인트 — 명령 수신 및 미션 분배
    │   ├── drone_controller.py        #   MAVLink FC 제어 (ARM, 이륙, GPS 이동, 착륙)
    │   ├── Fire_detector.py           #   YOLOv8s TensorRT 화재/연기 탐지 및 GPS 추정
    │   ├── Fire_extinguishing.py      #   소화볼 투하 미션 시퀀스
    │   ├── patrol.py                  #   자율 순찰 (경로점 이동 + 실시간 화재 감시)
    │   ├── Landing.py                 #   빨간 마커 추적 기반 정밀 착륙
    │   ├── DroneCommunicator.py       #   PC 관제 센터 TCP 통신 (보고/명령)
    │   ├── servo_control.py           #   소화볼 투하 서보 PWM 제어
    │   └── pipeline.py                #   GStreamer CSI 카메라 파이프라인
    │
    ├── PC/                            # PC 관제 센터 소프트웨어
    │   ├── pc_server.py               #   Tkinter 관제 GUI (지도, 경로, 화재 대응)
    │   ├── pc_rpi_server.py           #   Station(RPi) TCP 통신 서버
    │   └── pc_server.exe              #   빌드된 실행 파일
    │
    └── Station_WIP/                   # 드론 스테이션 [개발 중단 / Work In Progress]
        ├── station.py                 #   스테이션 메인 — 서보, ToF 센서, 도킹 제어
        ├── Doc_match.py               #   카메라 기반 도킹 마커 감지
        └── rpi_client.py              #   PC 서버 TCP 통신 클라이언트
```

## 소프트웨어 모듈 상세

### Drone (Jetson Orin Nano)

| 모듈 | 설명 |
|------|------|
| `main.py` | 시스템 진입점. PC로부터 TCP 명령을 수신하여 순찰(list) 또는 진압(EXTINGUISH) 미션을 스레드로 분배 |
| `drone_controller.py` | pymavlink 기반 FC 제어. GUIDED/RTL/LAND/BRAKE 모드 전환, ARM/DISARM, GPS 이동(goto_gps), 하버사인 거리 계산 |
| `Fire_detector.py` | YOLOv8s TensorRT 엔진으로 정면 카메라 화재/연기 탐지. 5초 30프레임 확정 로직, 카메라 각도 기반 화재 GPS 추정, 하단 카메라 객체 정렬 |
| `patrol.py` | 경로점 순회 + 실시간 화재 감시 스레드 병행. 화재 확정 시 화재 지점 이동 → 정렬 → 사진 촬영 → 홈 복귀 → 보고 전송 |
| `Fire_extinguishing.py` | 화재 지점 GPS 이동 → 하단 카메라 정렬 → 서보 180도 회전으로 소화볼 투하 → 홈 복귀 → 완료 보고 |
| `Landing.py` | 하단 카메라 빨간 마커(Red Dot) HSV 감지. 비례 제어로 마커 중앙 정렬하며 하강, 10초 미감지 시 LAND 모드 전환 |
| `DroneCommunicator.py` | PC 관제 센터로 화재 보고(좌표+이미지) 및 상태 업데이트 TCP 전송. 포트 65523에서 명령 수신 서버 운영 |
| `servo_control.py` | Jetson GPIO PWM 서보 제어. 0도=재장전(idle), 180도=투하 |
| `pipeline.py` | IMX477(30fps 1920x1080) / IMX219(21fps 1640x1232) 센서별 GStreamer 파이프라인 생성 |

### PC 관제 센터

| 모듈 | 설명 |
|------|------|
| `pc_server.py` | Tkinter + TkinterMapView 지도 GUI. 우클릭 경로점 추가, 순찰 명령 전송, 화재 보고 수신 시 지도 핑 + 사진 표시 + 진압 승인 |
| `pc_rpi_server.py` | Station(RPi)과의 TCP 서버. 소화볼 장전 명령 전송 및 응답 수신. 포트 65525 |

### Station [WIP — 개발 중단]

| 모듈 | 설명 |
|------|------|
| `station.py` | 서보 3축(전후/좌우/장전) + ToF 센서 4개(VL53L0X)로 드론 도킹 및 소화볼 장전 자동화 |
| `Doc_match.py` | CSI 카메라 빨간 마커 감지 (원형도/종횡비 필터링) 기반 도킹 정렬 |
| `rpi_client.py` | PC 서버 TCP 클라이언트. 명령 수신 큐 + JSON 응답 전송, 자동 재연결 |

## 하드웨어 사양

| 구성 요소 | 사양 |
|-----------|------|
| **개발보드** | NVIDIA Jetson Orin