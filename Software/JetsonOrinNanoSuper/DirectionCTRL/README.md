# Drone UART Connection & Control 
## == 순찰 need Input data == (List)
- [vertical, horizontal1, horizontal2, rotation, speed] 형식
- drone.set_command(vertical, horizontal, rotation, speed)
- vertical = (up, level, down (택1)
- horizontal1 = (forward, 0, backward) (택1)
- horizontal2 = (left, 0, right) (택1)
- L3 = 드론 정면 방향 기준 시계방향 회전 (0~359 degree)
- L4 = 진행 거리 지정
- 이륙 throttle = 90%, 호버링 throttle = 60%, 착륙 throttle = 50%
- 총 4가지 데이터를 받아서 FC에 전송

- gps데이터 (위도,경도,(고도 or None)) 를 받으면  드론 이동

