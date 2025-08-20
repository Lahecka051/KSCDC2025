# Drone UART Connection & Control 
## == 순찰 need Input data == (List)
- (up, level, down (택1)),(forward,backward, / left,right, / hover) >>> forward_left 형태로 2가지 사용해서 응용가능
- 드론 정면 방향 기준 시계방향 회전 (0~359 degree)
- 속도(m/s or 모터 부하 (0~100%))
- 총 4가지 데이터를 받아서 FC에 전송

- gps데이터를 받으면
- (위도,경도)를 바탕으로 드론 이동

## == 화재 진압 need Input data == (dictionary)


## 드론스테이션 
- 리턴투홈 착륙중에 착륙 완료 판단 유무 정확한 확인 필요

## (필요한 추가사항)
- FC의 고도센서의 고도값을 일정하게 유지, if 산 위를 비행할때 ALT값이 낮아지면 지정된 고도값 ex) 10m를 유지하면서 비행
- 
