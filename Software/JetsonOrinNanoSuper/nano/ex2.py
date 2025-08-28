import queue, threading

result_q = queue.Queue()

# 스레드 실행
threading.Thread(
    target=self.fire_detection_thread,
    args=(drone_gps, result_q),
    daemon=True
).start()

# 메인 루프에서 제어
while True:
    try:
        result = result_q.get_nowait()
        if result["status"] == "started":
            print("➡ 이동 멈춤, 화재 관찰 시작")
        elif result["status"] == "recognized":
            print("✅ 화재 인식 완료, 좌표:", result["coords"])
            print("➡ 출발지로 복귀")
            break
        elif result["status"] == "failed":
            print("❌ 인식 실패 → 다시 지정 좌표로 이동")
    except queue.Empty:
        print("➡ 이동 중...")
        time.sleep(0.5)
