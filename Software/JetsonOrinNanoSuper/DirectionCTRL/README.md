# Drone UART Connection & Control 
## == need Input data ==
- (up, level, down (택1)),(forward,backward, / left,right, / hover) >>> forward_left 형태로 2가지 사용해서 응용가능
- 드론 정면 방향 기준 시계방향 회전 (0~359 degree)
- 속도(m/s or 모터 부하 (0~100%))
- 총 4가지 데이터를 받아서 FC에 전송

# CTRL Logic
{
    "type": "CTRL",
    "seq": 123,
    "timestamp": "2025-01-15T10:30:00",
    "command": {
        "vertical": "up",        // up/level/down
        "horizontal": "forward_left",  // 8방향 + hover
        "rotation": 45,          // 0-359도
        "speed": 50,            // 속도값
        "speed_type": "percent"  // percent 또는 m/s
    }
}

# GPS Logic
{
    "type": "GPS",
    "timestamp": "2025-01-15T10:30:00",
    "data": {
        "position": {
            "lat": 35.123456,
            "lon": 129.123456,
            "alt_abs": 100.5,
            "alt_rel": 2.5
        },
        "velocity": {
            "north": 1.2,
            "east": 0.5,
            "down": -0.1
        },
        "attitude": {
            "roll": 0.5,
            "pitch": -1.2,
            "yaw": 45.0
        },
        "battery": {
            "voltage": 12.4,
            "percent": 85
        }
    }
}
