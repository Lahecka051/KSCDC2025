# Software

KSCDC2025 프로메테우스 시스템 소프트웨어 구성. 3개 노드(PC 관제 / 드론 / 스테이션)로 구성된 분산 시스템.

```
PC 관제 (Tkinter GUI)  ───TCP───  Drone (Jetson Orin Nano)
        │
        └────TCP────  Station (Raspberry Pi, WIP)
```

## Drone/ — 드론 탑재 (Jetson Orin Nano Super)

YOLOv8s + TensorRT 기반 화재/연기 실시간 탐지 및 자율 비행/투하.

### 실행

```bash
cd Software/Drone
python3 main.py
```

### 의존성

- Python 3.10
- pymavlink, OpenCV (CUDA), TensorRT, ultralytics
- Jetson.GPIO
- GStreamer + nvarguscamerasrc (CSI 카메라)

### 진입점

`main.py` — PC 관제로부터 TCP 명령 수신 후 미션 분배 (순찰/진압).

## PC/ — 관제 센터 (Windows/Linux)

Tkinter + TkinterMapView 지도 GUI. 경로점 설정, 화재 보고 수신, 진압 승인.

### 실행

```bash
cd Software/PC
python pc_server.py
```

또는 빌드된 `pc_server.exe` 실행 (Windows).

### 의존성

- Python 3.10
- tkinter, tkintermapview, Pillow

## Station_WIP/ — 자동 장전 스테이션 [개발 중단]

Raspberry Pi 기반 자동 도킹/장전 시스템. 시간 부족으로 본 대회에서는 미투입, 향후 확장 작업용.

### 구성

- 서보 3축(전후 / 좌우 / 장전 회전)
- ToF 센서 4개 (VL53L0X) — 도킹 거리 측정
- CSI 카메라 — 빨간 마커 도킹 정렬

## 통신 포트

| 노드 | 수신 포트 | 용도 |
|------|----------|------|
| 드론 | 65523 | PC → 드론 명령 |
| PC | 65524 | 드론 → PC 보고 |
| Station | 65525 | PC → Station 장전 명령 |
