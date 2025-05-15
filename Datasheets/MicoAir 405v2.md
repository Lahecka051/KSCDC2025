# MicoAir405v2 비행 컨트롤러 

- 고성능 IMU가 내장된 비용 효율적인 비행 컨트롤러로 Ardupilot 및 INAV를 지원합니다. 
- Ardupilot v4.4.4 가 기본 설치되어 있으며, v4.5 부터 공식 파트너로 미션플래너/QGC에서 업데이트 가능합니다. 
- 제품에 대한 더욱 자세한 내용과 다운로드는 아래의 본사 링크를 참고하세요.
- https://micoair.com/index.php/flightcontroller_micoair405v2/
 
## 상세 제원 MCU : STM32F405RGT6, 168MHz, 1MB 플래시 
- IMU: BMI088 
- 바로: SPLO6 
- OSD: AT7456E 
- MicroSD 카드 슬롯 
- 6x UART 
- 10x PWM 
- 1xI2C
- 1xSWD
- 2xADC(VBAT,전류)
- USB 타입-C
- BEC 5V 3A 출력(컨트롤러, 수신가, GPS, 광학 흐름 또는 기타 장치용)
- BEC 9V 3A 출력(비디오, 송신기, 카메라용)

## UART 매핑
- 직렬0 -> USB
- ERIAL1 -> UART1(DMA 지원)
- SERIAL2 -> UART2(DJI-VTX, DMA 지원)
- 직렬3 -> UART3(GPS) 
- SERIAL4 -> UART4(DMA 지원) 
- SERIAL5 -> UART5(ESC 원격 측정)
- SERIAL6 -> UART6(RX6은 SBUS 핀에서 반전되고 TX6에는 DMA가 없음)
 
## RC 입력 
- 기본 RC 입력은 SBUS 판에서 반전된 UART6 RX에국 구성됩니다.
- 다른 RC 프로토콜은 UART1 또는 UART4 와 같은 다른 UART 포트에 적용되어야 하며 RC 데이터를 수신하도록 프로토콜을 설정해야 합니다.
- SERIALn_PROTOCOL=23 및 SERIAL6_Protocol을 23' 이외의 다른 것으로 변경 

## OSD 지원 
- MicoAir405v2는 OSD_TYPE 1(MAX7456 드라이버)을 사용하여 OSD를 지원합니다. 

## VTX 지원
- VTX 지원 SH1.0-6P 커넥터는 DJI 03 에어 유닛 연결을 지원합니다. 커넥터의 핀 1은 9V이므로 5V/가 필요한 주변 장치에 연결하지 않도록 주의하세요.

## PWM 출력 
- MicoAir405v2는 최대 10개의 PWM 출력을 지원합니다.
- 채널 1-8은 DShot을 지원합니다.
- 채널 1-4는 양방향 DShot을 지원합니다.
- PWM 출력 공유는 그룹화되어 있으며 모든 그룹은 동일한 출력 프로토콜을 사용해야 합니다.
- 1,2,5,6은 그룹 1입니다. 3,4는 그룹 2입니다. 7,8은 그룹 3입니다. 9,10은 그룹 4에 속해 있습니다.

## 배터리모니터링 
- 보드에는 내부 전압 센서가 있으며 외부 전류 센서 입력을 위한 ESC 커넥터 연결이 있습니다. 
- 전압 센서는 최대 6S LiPo 배터리를 처리할 수 있습니다. 
### Ardupilot의 기본 배터리 매개변수는 다음과 같습니다.
- BATT_VOLT_PIN 10 
- BATT_CURR PIN 11 
- BATT_VOLT_MULT 21.2 
• BATT_CURR_SCALE 40.2

## 나침반 
- MicoAir405v2에는 내장 나침반이 없지만 SDA 및 SCL 커넥터에서 12C를 사용하여 외부 나침반을 연결할 수 있습니다.

## 물리적 
- 장착: 30.5 30.5mm, 4mm
- 크기:36 x36 8mm 
- 무게: 9g

![Image](https://github.com/user-attachments/assets/816e35b1-ded0-4108-8cde-cd1841832b99)
![Image](https://github.com/user-attachments/assets/b2d11347-9f01-49e2-88be-c07ad6bfc129)
![Image](https://github.com/user-attachments/assets/8962122d-c5a3-4c06-abb7-30e61a7902d0)
![Image](https://github.com/user-attachments/assets/9f3f3601-4f2e-432f-8294-2454f6ded633)
![Image](https://github.com/user-attachments/assets/527aab19-54d3-4a07-ac1c-360e843c29fc)
