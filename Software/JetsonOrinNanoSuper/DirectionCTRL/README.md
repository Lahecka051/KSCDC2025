# Drone UART Connection & Control 
## == 순찰 need Input data == (List)
- [vertical, horizontal, rotation, speed] 형식
- drone.set_command("L1", "L2", L3, L4)
- L1 = (up, level, down (택1)
- L2 = ((forward,backward, / left,right), hover) >>> forward_left 형태로 2가지 사용해서 응용가능
- L3 = 드론 정면 방향 기준 시계방향 회전 (0~359 degree)
- L4 = 모터 부하 (0~100%)
- 이륙 throttle = 90%, 호버링 throttle = 60%, 착륙 throttle = 50%
- 총 4가지 데이터를 받아서 FC에 전송

- gps데이터 (위도,경도,(고도 or None)) 를 받으면  드론 이동

