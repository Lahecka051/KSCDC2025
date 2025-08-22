# Jetson Orin Nano용 완성 코드

### main.py
- 실제로 실행될 main 각종 객체생성 초기값 설정 등등
### DroneCommunicator.py
- PC 관제 센터와의 모든 '클라이언트' 통신을 전담
  - send_fire_report(coordinates, image_path) : PC 관제 센터로 화재 보고(좌표, 이미지)를 전송
  - send_status_update(status_message) : PC 관제 센터로 간단한 상태 메시지를 전송 ex) 착륙완료 신호
  - start_receiver() : 수신 받을 준비
  - receive_data() : 명령을 수신 받는 함수
 
### pipeline.py
- CSI카메라 사용을 위한 pipeline 생성 함수
- 해상도 1280x720 프레임사이즈 960x960
