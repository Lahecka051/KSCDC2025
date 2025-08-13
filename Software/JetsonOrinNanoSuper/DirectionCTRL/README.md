# Drone UART Connection & Control 
## == need Input data ==
- (up, level, down (택1)),(forward,backward, / left,right, / hover) >>> forward_left 형태로 2가지 사용해서 응용가능
- 드론 정면 방향 기준 시계방향 회전 (0~359 degree)
- 속도(m/s or 모터 부하 (0~100%))
- 총 4가지 데이터를 받아서 FC에 전송
