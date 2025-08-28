def fire_detection_thread(self, drone_gps, result_q):
    """
    스레드에서 실행되는 화재 감지 루프
    - 감지 시작시: {"status": "started"}
    - 4초 연속 감지 완료시: {"status": "recognized", "coords": (lat, lon)}
    - 중간에 끊기면: {"status": "failed"}
    """
    detecting = False
    start_time = None
    detection_count = 0
    last_coords = None

    while True:
        ret, frame = self.cap0.read()
        if not ret:
            continue

        results = self.model.predict(frame, imgsz=920, conf=0.4, verbose=False)

        fire_detected = False
        center_x, center_y = None, None

        if len(results[0].boxes) > 0:
            best_box = max(results[0].boxes, key=lambda box: box.conf[0])
            class_name = self.class_names[int(best_box.cls[0])]
            if class_name.lower() in ["fire", "smoke"]:
                fire_detected = True
                x1, y1, x2, y2 = best_box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

        if fire_detected:
            coords = self.fire_gps(drone_gps, center_x, center_y)
            last_coords = coords

            if not detecting:
                detecting = True
                start_time = time.time()
                detection_count = 1
                result_q.put({"status": "started"})   # 🚨 감지 시작 알림
            else:
                detection_count += 1

            elapsed = time.time() - start_time
            if elapsed >= self.OBSERVATION_TIMEOUT:  # 4초 관찰
                if detection_count >= 3:  # 최소 감지 횟수 조건
                    result_q.put({"status": "recognized", "coords": last_coords})
                else:
                    result_q.put({"status": "failed"})
                detecting = False
                detection_count = 0
                start_time = None

        else:
            if detecting:
                # 감지 중이었는데 끊김 → 실패 판정
                result_q.put({"status": "failed"})
                detecting = False
                detection_count = 0
                start_time = None
