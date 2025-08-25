# Jetson Orin Nano용 코드

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

### Fire_detector.py
- 화제 탐지 및 화제현장바로 위로 정렬 담당
- fire_gps(drone_gps, center_x, center_y) : 현제 위치를 기반으로 화제 위치 계산
- detect_fire_upper(drone_gps) : 순찰시 객체 인식을 담당
- patrol_logic() : 화제지점 주변 정찰 사각형 모양으로 순찰
- align_drone_to_object() : 화제 바로 위로 정렬 시키는 함수
- capture_and_save_image(output_path="captured_image.jpg") : 아래쪽 카메라로 사진을 찍어 지정된 경로에 저장
