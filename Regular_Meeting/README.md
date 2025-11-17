# == 정기 회의록 ==
- KSCDC2025 팀 프로젝트 회의 진행중 나온 의견을 정리하는 장소입니다.
- 메인 페이지의 정리된 회의 내용은 회의 종료 후 2일 이내 업데이트 됩니다.
- 자세한 사항은 당일 회의록을 참고하시기 바랍니다.

# Paste_Data : 09.11(목) (latest update) 

### 정기 회의일
- 매주 (수,목)요일 14:00 ~ 20:00

## 논의 내용
- 하드웨어 & 소프트웨어 제작

## 진행 상황
### 소프트웨어
- 젯슨 CSI 카메라 객체인식 테스트 YOLOv8s
- FC의 데이터를 텔레메트리를 이용하여 실시간 모니터링 가능한 지상국 구축
- FC의 모터 PWM 신호 애널라이징 후 신호 복제하여 무선 모터 조종 테스트

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
- 정면 : IMX477 6mm 63 degree
- 바닥 : IMX219 160 degree

#### 배터리 
- LiPo 4S(14.8V) 8,400mAh 130C X2 
- IMAX B6 LiPo 충전기
- [배터리 구매 링크](https://ko.aliexpress.com/item/1005001956377380.html?spm=a2g0o.order_list.order_list_main.131.f19d140fXWRv7X&gatewayAdapt=glo2kor)
- [충전기 구매 링크](https://ko.aliexpress.com/item/1005005231331856.html?spm=a2g0o.order_list.order_list_main.125.f19d140fXWRv7X&gatewayAdapt=glo2kor)

#### 개발보드 
- Jetson Orin Nano Super
- [개발보드 공식 링크](https://www.nvidia.com/ko-kr/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

### 소프트웨어
- UBUNTU_GUI 22.04 LTS (Jetpack6.2.1)
- Ardupilot 5.4.2
- USED OPENCV CUDA TensorRT
- YOLO MODEL : YOLOv8s

## 프레임
- 페이로드 : 3kg 이하, 사이즈 : 300x300 mm 이상
- X6-X600 FPV 프레임 600mm / 30A ~ 40A / 2212 920KV 모터
- [프레임 구매 링크](https://ko.aliexpress.com/item/1005008274592701.html?spm=a2g0o.order_list.order_list_main.137.f19d140fXWRv7X&gatewayAdapt=glo2kor)

### 부품 공식 제원 상 무게
- jetson orin nano super : Dimensions = 100mm x 79mm x 21mm, Weight = 176g, Power = 5V_Input(5W ~ 25W)
- Camera : 수정
- Flight Controller Spec : Mounting = 30.5mm x 30.5mm x 4mm, Dimensions = 36mm x 36mm x 8mm, Weight = 10g
- 14.8V 8400mah 130C 리포 하드케이스 크기: 138 x 47 x 49mm 무게 639g.
- 소화볼 규격: 지름 4inch(약 100mm), 약 400g
- [소화볼 규격 링크](https://fireds.com/product/%EC%BD%94%EB%81%BC%EB%A6%AC%EC%86%8C%EB%B0%A9%EB%A7%88%ED%8A%B8-%ED%99%94%EC%9D%B4%EC%96%B4%EB%B3%BC4%EC%9D%B8%EC%B9%98-%ED%99%94%EC%9E%AC%EC%B4%88%EA%B8%B0-%EC%9E%90%EB%8F%99-%EC%A7%84%EC%95%95%EA%B3%B5-%EC%86%8C%EA%B3%B5%EA%B0%84-%EC%9E%90%EB%8F%99%EC%86%8C%ED%99%94%EC%9E%A5%EC%B9%98-%EC%9E%90%EB%8F%99%EC%86%8C%ED%99%94%EA%B8%B0/8683/)
## 앞으로 진행할 내용
- 젯슨 객체인식 소프트웨어 & 지상국 소프트웨어 구체화
