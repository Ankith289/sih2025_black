import cv2
import threading
import time
import os
from ultralytics import YOLO
from datetime import datetime

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
    """Save image with missing fastener detection"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = f"{class_name}_{timestamp}_{confidence:.2f}.jpg"
    filepath = os.path.join(SAVE_PATH, filename)
    
    # Crop the region around the fastener with padding
    x1, y1, x2, y2 = map(int, box)
    padding = 20
    cropped = frame[max(0, y1-padding):min(frame.shape[0], y2+padding), 
                    max(0, x1-padding):min(frame.shape[1], x2+padding)]
    
    # Save the cropped image
    cv2.imwrite(filepath, cropped)
    print(f"Missing fastener detected and saved: {filepath}")

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
                save_bad_fastener(frame, (x1, y1, x2, y2), conf, class_name)
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
