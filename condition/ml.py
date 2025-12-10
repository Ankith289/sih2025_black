import cv2
import threading
import time
from ultralytics import YOLO

# =====================================================================
# Configuration
# =====================================================================

MODEL_PATH = "best.pt"
INPUT_SIZE = 512      # Reduce from 832 → 512 for faster inference
SKIP_FRAMES = 1       # Process every frame (set 2 for more speed)
DRAW_BOXES = True

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

    # Resize for faster inference
    resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))

    # YOLO inference (CPU-only)
    result = model(resized, verbose=False, conf=0.4)[0]

    # =================================================================
    # Bounding box drawing (fast OpenCV version)
    # =================================================================
    if DRAW_BOXES:
        boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()

        for box, cls, conf in zip(boxes, classes, confs):
            x1, y1, x2, y2 = map(int, box)
            label = f"{model.names[int(cls)]} {conf:.2f}"

            # Draw bounding box
            cv2.rectangle(resized, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw text
            cv2.putText(resized, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 2)

    # =================================================================
    # FPS counter
    # =================================================================
    now = time.time()
    fps = 1 / (now - prev_time)
    prev_time = now

    cv2.putText(resized, f"FPS: {fps:.2f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    cv2.imshow("YOLO PT (Bounding Boxes)", resized)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

stream.stop()
cv2.destroyAllWindows()
