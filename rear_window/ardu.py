import cv2
import threading
import time
import os

# -------------------------------
# Configuration
# -------------------------------
CAMERA_IDS = [0]                      # MIPI CSI camera ID
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 30

WEB_FRAME_PATH = "/home/nvidia/rail-inspection-app/public/rear_window/frame.jpg"
VIDEO_OUTPUT_PATH = "rear_window_recording.mp4"

latest_frames = {}
frames_lock = threading.Lock()
stop_event = threading.Event()

# -------------------------------
# GStreamer Pipeline
# -------------------------------
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
        % (
            sensor_id, capture_width, capture_height,
            framerate, flip_method,
            display_width, display_height
        )
    )

# -------------------------------
# Camera Thread
# -------------------------------
def capture_thread_func(sensor_id):
    print(f"[INFO] Starting camera {sensor_id} capture thread...")

    pipeline = gstreamer_pipeline(
        sensor_id=sensor_id,
        capture_width=FRAME_WIDTH,
        capture_height=FRAME_HEIGHT,
        display_width=FRAME_WIDTH,
        display_height=FRAME_HEIGHT,
        framerate=FRAME_RATE
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {sensor_id}")
        return

    # -------------------------------
    # Video Recorder (Jetson HW Encoder)
    # -------------------------------
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(
        VIDEO_OUTPUT_PATH, fourcc, FRAME_RATE, (FRAME_WIDTH, FRAME_HEIGHT)
    )

    if not video_writer.isOpened():
        print("[WARNING] Video writer failed to open. Video will NOT be saved.")

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print(f"[WARN] Camera {sensor_id}: No frame received.")
            break

        # Update latest frame for display
        with frames_lock:
            latest_frames[sensor_id] = frame

        # Save to web app
        try:
            cv2.imwrite(WEB_FRAME_PATH, frame)
        except:
            pass

        # Save video
        try:
            if video_writer.isOpened():
                video_writer.write(frame)
        except:
            pass

    cap.release()
    if video_writer.isOpened():
        video_writer.release()

    print(f"[INFO] Camera {sensor_id} thread stopped.")


# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    print("[SYSTEM] Starting rear-window subsystem...")

    # Ensure web app frame directory exists
    os.makedirs(os.path.dirname(WEB_FRAME_PATH), exist_ok=True)

    threads = []

    for cam_id in CAMERA_IDS:
        t = threading.Thread(target=capture_thread_func, args=(cam_id,))
        t.start()
        threads.append(t)

    print("[SYSTEM] Capture threads running. Video recording started.")
    print("[SYSTEM] Press CTRL+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[SYSTEM] Shutting down...")

    stop_event.set()

    for t in threads:
        t.join()

    print("[SYSTEM] All threads stopped. Rear window subsystem terminated.")
