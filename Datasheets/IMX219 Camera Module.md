# IMX219 160도 광각 카메라 모듈 8MP

## 사양
- 인터페이스 : CSI
- 해상도 : 3280 × 2464
- 크기 : 25mm × 24mm
- 케이블 길이 : 약 16cm
- 4개의 나사 구멍으로 3.3V의 직류 공급이 가능합니다.

## 렌즈 사양
- CMOS 크기 : 1/4인치
- 조리개 : F2.35
- 초점 거리 : 3.15mm
- 화각 : 160°
- 왜곡 : <14.3%

## 구성품
- 카메라모듈 1개
- 15핀 FFC 케이블 1개
 
## 카메라 테스트 명령어
DISPLAY=:0.0 gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1' ! nvoverlaysink -e

## 화상이 빨갛게(자주빛으로) 보일 때
- wget https://www.waveshare.com/w/upload/e/eb/Camera_overrides.tar.gz
- tar zxvf Camera_overrides.tar.gz
- sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/
- sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
- sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
