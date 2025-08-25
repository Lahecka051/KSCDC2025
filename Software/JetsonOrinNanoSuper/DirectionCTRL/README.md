# Drone UART Connection & Control 
## == 순찰 need Input data == (List)
- [vertical, horizontal1, horizontal2, rotation] 형식
- drone.set_command(vertical, horizontal, rotation, speed)
- vertical = (up, 0, down) (택1)  (+)값일때 up (+)값 m/s, (-)값일때 down (-)값 m/s
- horizontal1 = (forward, 0, backward) (택1) (+)값일때 forward (+)값 m/s, (-)값일때 backward (-)값 m/s
- horizontal2 = (left, 0, right) (택1) (+)값일때 left (+)값 m/s, (-)값일때 right (-)값 m/s
- 만약 horizontal1 == horizontal2 == 0 면 hover
- rotation = 드론 정면 방향 기준 시계방향 회전 (0~359 degree)
- 총 4가지 데이터를 받아서 FC에 전송

- gps데이터 (위도,경도,(고도 or None)) 를 받으면  드론 이동

- 참고 github
- https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py

