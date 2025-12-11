import cv2
import threading
import time
import os
from ultralytics import YOLO
from datetime import datetime
import serial
import re
import sys
import zmq

# ============================
# GLOBAL TIMESTAMP READER
# ============================
TS_FILE = "/tmp/global_timestamp.txt"
_last_ts = "NO_TIMESTAMP"
_last_read = 0

def get_global_timestamp():
    global _last_ts, _last_read
    now = time.time()
    if now - _last_read >= 0.005:
        try:
            with open(TS_FILE, "r") as f:
                _last_ts = f.read().strip()
        except:
            _last_ts = "NO_TIMESTAMP"
        _last_read = now
    return _last_ts

# ============================
# SYNC PACKET BUILDER
# ============================
def build_fastener_packet(image_path, confidence, velocity):
    return {
        "timestamp": get_global_timestamp(),
        "subsystem": "track_component",
        "component_type": "fastener",
        "status": "defective",
        "defect_type": "missing_fastener",
        "confidence": round(confidence, 3),
        "image_path": image_path,
        "velocity": velocity   # optional
    }

# ============================
# UART CONFIG
# ============================
# ============================
# UART ZMQ SUBSCRIBER (from Time Master)
# ============================
MASTER_ADDR = "tcp://127.0.0.1:6000"

def uart_zmq_worker():
    """
    Listens to parsed UART data published by the UART Time Master.
    """
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(MASTER_ADDR)

    # Subscribe to ALL messages
    sub.setsockopt_string(zmq.SUBSCRIBE, "")

    sys.stderr.write(f"[UART-ZMQ] Connected to UART Master at {MASTER_ADDR}\n")

    while True:
        try:
            msg = sub.recv_json()   # blocking read

            # Expected structure:
            # { "d1":int, "d2":int, "velocity":float, "trigger":int, "distance":float, "timestamp":str }

            uart_state.update({
                "D1": msg.get("d1", 0),
                "D2": msg.get("d2", 0),
                "Velocity": msg.get("velocity", 0.0),
                "Trigger": msg.get("trigger", 0),
                "Distance": msg.get("distance", 0.0)
            })
        
        except Exception as e:
            sys.stderr.write(f"[UART-ZMQ] Error: {e}\n")
            time.sleep(0.1)

# =====================================================================
# Configuration
# =====================================================================

MODEL_PATH = "best.pt"
INPUT_SIZE = 512      # Reduce from 832 → 512 for faster inference
SKIP_FRAMES = 1       # Process every frame (set 2 for more speed)
DRAW_BOXES = True
BAD_FASTENER_CLASS = "missing fastener"  # Class name for missing fasteners
SAVE_PATH = "/home/user/bad_fasteners"  # Linux path for saving images (change 'user' to your username)

# Create directory if it doesn't exist
os.makedirs(SAVE_PATH, exist_ok=True)

# =====================================================================
# Load YOLO model
# =====================================================================

model = YOLO(MODEL_PATH)
model.fuse()
print("Model loaded.")
threading.Thread(target=uart_zmq_worker, daemon=True).start()
# =====================================================================
# Threaded camera class
# =====================================================================

class CameraStream:
    def __init__(self, src=0, width=640, height=360):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(3, width)
        self.cap.set(4, height)
        self.ret = False
        self.frame = None
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.running = False
        self.cap.release()


# Start camera
stream = CameraStream()
time.sleep(1)

# =====================================================================
# Function to save missing fastener image
# =====================================================================

def save_bad_fastener(frame, box, confidence, class_name):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = f"{class_name}_{timestamp}_{confidence:.2f}.jpg"
    filepath = os.path.join(SAVE_PATH, filename)

    x1, y1, x2, y2 = map(int, box)
    padding = 20
    cropped = frame[max(0, y1-padding):min(frame.shape[0], y2+padding),
                    max(0, x1-padding):min(frame.shape[1], x2+padding)]

    cv2.imwrite(filepath, cropped)
    print(f"Missing fastener detected and saved: {filepath}")
    return filepath

# =====================================================================
# Inference loop
# =====================================================================

frame_id = 1
prev_time = time.time()

while True:
    ret, frame = stream.read()
    if not ret:
        continue

    frame_id += 1
    if frame_id % SKIP_FRAMES != 0:
        continue

    # Store original frame dimensions
    orig_height, orig_width = frame.shape[:2]

    # Resize for faster inference
    resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))

    # YOLO inference (CPU-only)
    result = model(resized, verbose=False, conf=0.4)[0]

    # Calculate scale factors
    scale_x = orig_width / INPUT_SIZE
    scale_y = orig_height / INPUT_SIZE

    # =================================================================
    # Bounding box drawing and missing fastener detection
    # =================================================================
    if DRAW_BOXES:
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()

        for box, cls, conf in zip(boxes, classes, confs):
            # Scale coordinates back to original frame size
            x1, y1, x2, y2 = map(int, box)
            x1 = int(x1 * scale_x)
            y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x)
            y2 = int(y2 * scale_y)

            class_name = model.names[int(cls)]
            label = f"{class_name} {conf:.2f}"

            # Check if missing fastener detected
            if class_name.lower() == BAD_FASTENER_CLASS.lower():
                # Draw red bounding box for missing fastener
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                # Save the missing fastener image
                filepath = save_bad_fastener(frame, (x1, y1, x2, y2), conf, class_name)
                # ===============================
                # ONLY SEND PACKET IF TRIGGER == 1
                # ===============================
                uart_data = uart_state.snapshot()
                
                trigger = uart_data.get("Trigger", uart_data.get("Triger", 0))
                velocity = uart_data.get("Velocity", uart_data.get("Vel", 0.0))
                
                if trigger == 1:
                    sync_packet = build_fastener_packet(filepath, conf, velocity)
                    print(json.dumps(sync_packet), flush=True)
                
                # Draw text with red background
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 255), 2)
            else:
                # Draw green bounding box for good fastener
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Draw text
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 2)

    # =================================================================
    # FPS counter
    # =================================================================
    now = time.time()
    fps = 1 / (now - prev_time)
    prev_time = now

    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    cv2.imshow("YOLO PT (Bounding Boxes)", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

stream.stop()
cv2.destroyAllWindows()
