def gstreamer_pipeline(
        sensor_id, 
        capture_width=None,
        capture_height=None,
        display_width=640,
        display_height=640,
        framerate=None,
        flip_method=0
    ):
    
    # 센서별 최적 설정
    if sensor_id == 0:  # IMX477
        capture_width = capture_width or 1920
        capture_height = capture_height or 1080
        framerate = framerate or 30
    else:  # IMX219 (sensor_id == 1)
        capture_width = capture_width or 1640
        capture_height = capture_height or 1232
        framerate = framerate or 21  # 중요: 21fps로 제한
    
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width=(int){capture_width}, "
        f"height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, "
        f"format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
    )