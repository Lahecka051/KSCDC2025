# Drone UART Connection & Control 
## == 순찰 need Input data == (List)
- [vertical, horizontal1, horizontal2, rotation] 형식
- set_command(vertical, horizontal, rotation, speed)
- vx = cmd[0]  # 전진/후진 (North)
- vy = -cmd[1]  # 좌/우 (East, 좌측이 +이므로 부호 반전)
- vz = -cmd[2]  # 상승/하강 (Down, 상승이 +이므로 부호 반전)
- - 만약 vx == vy == 0 면 hover
- rotation = math.radians(int(cmd[3]))  # 드론 방향 (0~359도, 시계방향)

- 총 4가지 데이터를 받아서 FC에 전송

- gps데이터 (위도,경도,(고도 or None)) 를 받으면  드론 이동

## 참고 github
- https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py

