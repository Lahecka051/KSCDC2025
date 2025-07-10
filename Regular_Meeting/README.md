# == 정기 회의록 ==
- KSCDC2025 팀 프로젝트 회의 진행중 나온 의견을 정리하는 장소입니다.
- 메인 페이지의 정리된 회의 내용은 회의 종료 후 2일 이내 업데이트 됩니다.
- 자세한 사항은 당일 회의록을 참고하시기 바랍니다.

# Paste_Data : 07.0910(수,목) (latest update) 

### 정기 회의일
- 매주 (수,목)요일 14:00 ~ 21:00

## 진행 상황
- 프레임 조립 완료
- 소화탄 투하기 제작중
- FC 세팅, 딥러닝 & 모니터링 소프트웨어 제작중

### 기계공학적인 차별점이 필요
- 드론 하단부 소화탄 발사기 ( 소화탄 다연장 회전식 매커니즘 ) 탈부착
- 탈착일때 정찰용, 부착일때 화재진압용
- 드론스테이션 자동화 플랫폼

## 확정된 부품
- 예산 : 30만원 + @

#### 모터 : 이동희 교수님 
- Airgear 350 Combo Set Multi-Rotor UAV Power 4pcs Set X2
- 9.5 X 4.5 two blade Propeller
  
#### FC(Flight Controller)
- 미코에어 H743 V2 신형 픽스호크 비행 컨트롤러 (ARDUPILOT ONLY)
- [FC 구매 링크](https://ko.aliexpress.com/item/1005008824819033.html?spm=a2g0o.order_list.order_list_main.119.f19d140fXWRv7X&gatewayAdapt=glo2kor)

#### 카메라
- 정면 : IMX219 160 degree
- 바닥 : IMX219 120 degree

#### 배터리 
- LiPo 4S(14.8V) 8,400mAh 130C X2 
- IMAX B6 LiPo 충전기
- [배터리 구매 링크](https://ko.aliexpress.com/item/1005001956377380.html?spm=a2g0o.order_list.order_list_main.131.f19d140fXWRv7X&gatewayAdapt=glo2kor)
- [충전기 구매 링크](https://ko.aliexpress.com/item/1005005231331856.html?spm=a2g0o.order_list.order_list_main.125.f19d140fXWRv7X&gatewayAdapt=glo2kor)

#### 개발보드 
- Jetson Orin Nano Super
- [개발보드 공식 링크](https://www.nvidia.com/ko-kr/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

### 소프트웨어
- UBUNTU_GUI 24.2 & Debian_Linux 12
- ROS BASED Ardupilot
- TENSORFLOW BASED OPENCV
- MODEL : YOLOv8s

## 프레임
- 페이로드 : 3kg 이하, 사이즈 : 300x300 mm 이상
- X6-X600 FPV 프레임 600mm / 30A ~ 40A / 2212 920KV 모터
- [프레임 구매 링크](https://ko.aliexpress.com/item/1005008274592701.html?spm=a2g0o.order_list.order_list_main.137.f19d140fXWRv7X&gatewayAdapt=glo2kor)

### 부품 공식 제원 상 무게
jetson orin nano super : Dimensions = 100mm x 79mm x 21mm, Weight = 176g, Power = 5V_Input(5W ~ 25W)
Camera : 3g x 2 = 6g
Flight Controller Spec : Mounting = 30.5mm x 30.5mm x 4mm, Dimensions = 36mm x 36mm x 8mm, Weight = 10g


## 앞으로 진행할 내용
- 소프트웨어 데이터셋 작업 진행
