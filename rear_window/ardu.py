
import cv2
import threading
import time

# --- Configuration ---
# IMPORTANT: Verify these sensor-ids with `v4l2-ctl --list-devices`
CAMERA_IDS = [0]

# Video settings (adjust to your camera's capabilities)
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 30

# --- Global variables for thread communication ---
latest_frames = {}
frames_lock = threading.Lock()
stop_event = threading.Event()

# --- GStreamer Pipeline Function ---
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    """
    Constructs a GStreamer pipeline for capturing video from a MIPI CSI camera on a Jetson.
    """
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# --- Camera Capture Thread Function ---
def capture_thread_func(sensor_id):
    """
    This function is executed by each thread to capture frames from one camera.
    """
    print(f"📹 Capture thread for Camera {sensor_id}: Starting...")

    # 1. Create GStreamer pipeline and VideoCapture object
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
        print(f"❌ ERROR: Capture thread for Camera {sensor_id}: Cannot open camera.")
        return

    # 2. Frame capture loop
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print(f"⚠️ WARNING: Capture thread for Camera {sensor_id}: No frame received. Exiting.")
            break

        # Use a lock to safely update the shared dictionary
        with frames_lock:
            latest_frames[sensor_id] = frame

    # 3. Release resources
    cap.release()
    print(f"⏹️ Capture thread for Camera {sensor_id}: Stopped.")


# --- Main Execution ---
if __name__ == "__main__":
    threads = []

    print("🚀 Starting all camera capture threads...")

    # Create and start a capture thread for each camera
    for cam_id in CAMERA_IDS:
        thread = threading.Thread(target=capture_thread_func, args=(cam_id,))
        threads.append(thread)
        thread.start()

    print("\n✅ All capture threads started. Main display loop is running.")
    print("Press 'q' in any camera window to quit.")

    # Main loop to display frames from all threads
    try:
        while True:
            # Check if any frames are available
            if not latest_frames:
                time.sleep(0.1)
                continue

            with frames_lock:
                # Iterate over a copy of the items to avoid issues if the dict changes
                for sensor_id, frame in list(latest_frames.items()):
                    if frame is not None:
                        window_name = f"Camera {sensor_id}"
                        cv2.imshow(window_name, frame)

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Quit key pressed. Shutting down...")
                break
    finally:
        # 1. Signal all threads to stop
        stop_event.set()

        # 2. Wait for all threads to complete
        for thread in threads:
            thread.join()

        # 3. Close all OpenCV windows
        cv2.destroyAllWindows()
        print("🎉 All threads terminated and windows closed. Program finished.")
