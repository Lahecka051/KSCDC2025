# MicoAir743 V2 비행 컨트롤러
- Ardupilot/PX4/INAV/Betaflight 를 지원하는 듀얼 IMU 및 나침반 센서가 내장된 고성능 H743 비행 컨트롤러 입니다. 
- 펌웨어/ 핀아웃 링크: https://micoair.com/flightcontroller_micoair743v2/
- 매뉴얼: https://micoair.com/docs/micoair743-user- manual/
- 기본적으로 Ardupilot 이 설치되어 있습니다. 
- *PX4 설치시 stm32cube Programmer 를 이용하여 bootloader (bin 또는 ef) 파일을 먼저 설치하시고, QGC 에서 px4 펌웨어를 설치하시면 됩니다. 
- *펌웨어 설치 방법 참고 : https://youtu.be/H7TsouQIBfg 
- ESC 가 8비트 Bluejay 에서 32비트 AM32 로 업그레이드 되었습니다.
- 납땜 전에 반드시 USB 를 먼저 꽂아서 정상 작동하는지 점검하십시오. 납땜 이후에는 어떤 경우에도 초기 불량 처리가 되지 않습니다.

## Ardupilot 셋팅시 참고 파라미터 > 
- SERVO1_FUNCTION : 34 (M2) 
- SERVO2_FUNCTION : 35 (M3) 
- SERVO3_FUNCTION : 36 (M4) 
- SERVO4_FUNCTION : 33 (M1) 
- MOT PWM -TYPE : 6( (Dshot 600)
- SERVO _DSHOT ESC : 3 (AM32) 
- SERVO_BLH_BDMASK : 15 (생략가능, 적용시 RPM체크 가능) SERVO_BLH_AUTO : 1 
- NTF_BUZZ_TYPES : 3 (생략 가능, 적용시 모터를 부저로 사용)

## 상세 제원 
- MCU : STM32H743VIH6, 480MHz, 2MB 플래시 
- IMU : BMI088/BMI270 
- 바로 : SPLO6 
- 나침반 : QMC5883L 
- OSD : AT7456E 
- MicroSD 카드 슬롯 
- 8 x UART 
- 11 x PWM 
- 1 x 12C 
- 1 x SWD 
- 2 x ADC(VBAT, 전류) 
- USB 타입-C
- DJI 03/04 VTX 커넥터
- BEC 5V 3A 출력(컨트롤러, 수신기, GPS, 광학 흐름 또는 기타 장치용) 
- BEC 12V 3A 출력(비디오 송신기, 카메라용) 
- Bluetooth 원격 측정(UART8에 내부적으로 연결됨, 통신 속도 115200)
- VBAT 입력 범위: 2-6S(6-27V)

## UART 매핑( Ardupilot )
- 직렬0 -> USB 
- SERIAL1 -> UART1 (MAVLink2,DMA 지원) 
- SERIAL2 -> UART2(DisplayPort,DMA 지원) 
- SERIAL3 -> UART3(GPS, DMA 지원) 
- SERIAL4 -> UART4(MAVLink2, DMA 지원) 
- SERIAL5 -> UART5(DMA 지원) 
- SERIAL6 -> UART6( RCIN,DMA-활성화됨) 
- SERIAL7-> UART7(ESC 원격 측정, DMA 지원) 
- SERIAL8 -> UART8(DMA 활성화됨, 온보드 Bluetooth Telemetry에 연결됨 )

## UART 매핑(PX4)
- ttyACMo -> USB 
- ttyS0 -> TEL1 -> UART1 
- ttyS1 -> GPS2 -> UART2 
- ttyS2 -> GPS1 -> UART3 
- ttyS3 -> TEL2 -> UART4 
- ttyS4 -> TEL3-> UART5 
- ttyS5 -> RC -> UART6 
- ttyS6 -> UART6 -> UART7 
- tyS7 -> TEL4/SERIAL4 -> UART8

## RC 입력( Ardupilot )
- UART6는 ArduPilot이 지원하는 모든 수신기 프로토콜과 호환됩니다. 
- PPM은 지원되지 않습니다. 
- SBUS/DSM/SRXL은 RX6 핀에 연결됩니다. 
- FPort는 TX6에 대한 연결이 필요합니다. 
- FPort 수신기를 참조하세요. 
- CRSF는 RX6뿐만 아니라 TX6 연결도 필요하며 자동으로 원격 촉정을 제공합니다. 
- SRXL2는 TX6에 연결해야 하며 자동으로 원격 측정을 제공합니다. SERIAL6_OPTIONS를 "4"로 설정합니다.
- 모든 UART는 ArduPilot의 RC 시스템 연결에도 사용할 수 있으며 PPM을 제외한 모든 프로토콜과 호환됩니다. 자세한 내용은 무선 제어 시스템을 참조하세요.

## OSD 지원(Ardupilot) 
- MicoAir743 V2는 0SD_TYPE 1(MAX7456 드라이버 )를 사용하여 온보드 0SD를 지원합니다.
- 동시에 DisplayPort OSD는 HD VTX 커넥터에서 사용할 수 있습니다.

## VTX 지원 
- SH1.0-6P 커넥터는 DJI 에어 유닛/HD VTX 연결을 지원합니다.
- 프로토콜의 기본값은 DisplayPort 입니다.
- 커넥터의 핀 1은 12V이므로 5V가 필요한 주변 장치에 연결하지 않도록 주의하십시오.

## PWM 출력 
- MicoAir743 V2은 최대 11개의 PWM 출력을 지원합니다. 
- 채널 1-8은 DShot 및 양방향 DShot을 지원합니다. (현재 PX4&INAV는 BDShot 기능을 지원하지 않습니다) 
- PWM 출력 공유는 그룹화되어 있으며 모든 그룹은 동일한 출력 프로토콜을 사용해야 합니다. 
- 1,2,3,4는 그룹 1입니다.
- 5,6은 그룹 2입니다.
- 7,8,11은 그룹 3입니다.
- 9.10은 그룹 4입니다. 
-  참고 : PWM11은 "LED" 핀입니다. 이것이 직렬 LED 사용으로 구성된 경우 PWM7, 8은 직렬 LED로만 사용할 수 있습니다.

## 배터리 모니터링 
- 보드에는 내부 전압 센서가 있고 ESC 커넥터에는 외부 전류 센서 입력을 위한 연결 장치가 있습니다. 
- 전압 센서는 최대 6S LiPo 배터리를 처리할 수 있습니다. 
- Ardupilot의 기본 배터리 매개변수는 다음과 같습니다. 
- BATT_VOLT_PIN 10  
- BATT_CURR_PIN 11
- BATT_VOLT-MULT 21.2
- BAATT_CURR_SCALE 40.2

## 나침반 
- MicoAir743 V2에는 나침반 센서(QMc5883L)가 내장되어 있으며 SDA 및 SCL 커넥터에서 12C를 사용하여 외부 나침반을 연결할 수도 있습니다.

## 규격 
- 장착: 30.5 x 30.5mm, $4mm 
- 크기: 36 36 x 8mm 
- 무게: 10g

![Image](https://github.com/user-attachments/assets/41443ef4-1ca4-43ff-b8cb-82975c05bcad)
![Image](https://github.com/user-attachments/assets/12725685-8bac-4f52-b66b-b203c411af67)
![Image](https://github.com/user-attachments/assets/0d627f48-cdcb-43db-8b7a-c6eb0386f21b)
