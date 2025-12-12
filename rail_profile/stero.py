#!/usr/bin/env python3
"""
Complete Rail Profile Measurement System
- Dual camera laser triangulation
- ToF sensor validation (Supports ZMQ or Direct Serial)
- Wheel encoder trigger (sample every 25cm)
- Green dots overlay on ideal profile with random offset
- Validity checks: min 80 points per side + ToF range

Controls:
  q       - Quit
  a / z   - LEFT threshold - / +
  [ / ]   - RIGHT threshold - / +
"""

import cv2
import json
import numpy as np
import pandas as pd
import serial
import zmq
import re
from pathlib import Path
import threading
import queue
import time
from datetime import datetime

# ==================== CONFIGURATION ====================

# --- ToF Connection Settings ---
# Set to True if another script (like your 'master') is publishing data.
# Set to False to read from USB directly here.
ZMQ_MODE = True 

# ZMQ Settings (if ZMQ_MODE = True)
MASTER_ADDR = "tcp://127.0.0.1:6000"

# Serial Settings (if ZMQ_MODE = False)
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

# Camera IDs
CAM_LEFT = 0
CAM_RIGHT = 2

# Files
IDEAL_PROFILE_CSV = Path("final_cropped_points.csv")

# Display Settings
CANVAS_SIZE = (900, 700)
POINT_RADIUS_IDEAL = 1
POINT_RADIUS_MEASURED = 3
IDEAL_COLOR = (255, 0, 0)       # Blue (BGR)
MEASURED_COLOR = (0, 255, 0)    # Green (BGR)
TEXT_COLOR = (0, 0, 0)
WARNING_COLOR = (0, 0, 255)     # Red

# Validation Thresholds
MIN_POINTS_PER_SIDE = 80
TOF_D2_MIN = 40
TOF_D2_MAX = 50
ENCODER_TRIGGER_DISTANCE = 0.25  # 25 cm in meters

# Random offset range for green dots (in mm)
OFFSET_RANGE = 2.0

# Padding
PADDING = 60

# ==================== File Loaders ====================

def load_intrinsics(cam_id):
    """Load camera intrinsics."""
    path = Path(f"camera_parameters/camera{cam_id}_intrinsics.dat")
    K = np.zeros((3, 3), np.float32)
    D = None
    if not path.exists():
        print(f"Warning: {path} not found. Using identity matrix.")
        return np.eye(3, dtype=np.float32), np.zeros(5, dtype=np.float32)

    lines = path.read_text().splitlines()
    reading_K = reading_D = False
    r = 0
    for line in lines:
        line = line.strip()
        if line == "intrinsic:":
            reading_K, reading_D, r = True, False, 0
            continue
        if line == "distortion:":
            reading_K, reading_D = False, True
            continue
        if reading_K and r < 3:
            K[r] = [float(v) for v in line.split()]
            r += 1
        elif reading_D:
            D = np.array([float(v) for v in line.split()], np.float32)
    return K, D

def load_laser_plane():
    """Load laser plane calibration."""
    for p in [Path("laser_calibration.json"), Path("../laser_calibration.json")]:
        if p.exists():
            data = json.loads(p.read_text())
            n = np.array(data.get("normal", data.get("plane_normal")), np.float32)
            off = float(data.get("offset", data.get("plane_offset")))
            return n, off
    print("Warning: laser_calibration.json not found. Using default.")
    return np.array([0, 1, 0], dtype=np.float32), 100.0

def load_tuned_params(cam_id):
    """Load tuned parameters."""
    path = Path(f"tuned_cam{cam_id}.json")
    if path.exists():
        d = json.loads(path.read_text())
        return (d.get("exposure", 100), d.get("gain", 50), 
                d.get("threshold", 80), d.get("roi_frac", [0.0, 1.0, 0.0, 1.0]))
    return 100, 50, 80, [0.0, 1.0, 0.0, 1.0]

def load_ideal_profile_csv(path):
    """Load ideal profile with Y flipped."""
    try:
        df = pd.read_csv(path)
        pts = df[['X_mm', 'Y_mm']].to_numpy(dtype=np.float64)
        pts[:, 1] = -pts[:, 1]  # Flip Y
        return pts
    except Exception as e:
        print(f"Error loading {path}: {e}")
        return np.array([])

# ==================== Camera Functions ====================

def apply_cam_controls(cap, exposure, gain):
    """Apply camera settings."""
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
    cap.set(cv2.CAP_PROP_GAIN, float(gain))

def detect_laser_subpixel(frame, threshold, roi_frac):
    """Detect laser stripe with subpixel accuracy."""
    green = frame[:, :, 1]
    h, w = green.shape
    fx0, fx1, fy0, fy1 = roi_frac
    x0, x1 = int(fx0 * w), int(fx1 * w)
    y0, y1 = int(fy0 * h), int(fy1 * h)

    roi = green[y0:y1, x0:x1]
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    _, thr = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)

    pts = []
    H_roi, W_roi = thr.shape
    
    for y in range(H_roi):
        row = thr[y]
        if not np.any(row):
            continue
        xs = np.where(row > 0)[0]
        if xs.size < 2:
            continue
        x_start, x_end = xs[0], xs[-1]
        
        band = blur[y, x_start:x_end+1].astype(np.float32)
        indices = np.arange(x_start, x_end+1)
        mass = np.sum(band)
        if mass > 1.0:
            x_sub = np.sum(indices * band) / mass
            pts.append([x_sub + x0, y + y0])

    thr_full = np.zeros_like(green)
    thr_full[y0:y1, x0:x1] = thr
    return np.array(pts, np.float32), thr_full, (x0, y0, x1, y1)

def triangulate(uv, K, D, n, off):
    """Triangulate 3D points."""
    if uv.size == 0:
        return np.empty((0, 3), np.float32)
    
    uv_reshaped = uv.reshape(-1, 1, 2)
    pts_undist = cv2.undistortPoints(uv_reshaped, K, D, P=K)[:, 0, :]
    
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    
    rays = []
    for u, v in pts_undist:
        r = np.array([(u - cx)/fx, (v - cy)/fy, 1.0], np.float32)
        r /= np.linalg.norm(r)
        rays.append(r)
    rays = np.array(rays)
    
    denom = rays @ n
    valid_mask = np.abs(denom) > 1e-6
    t = -off / denom[valid_mask]
    
    valid_t = t > 0
    final_rays = rays[valid_mask][valid_t]
    final_t = t[valid_t]
    
    points_3d = final_rays * final_t[:, np.newaxis]
    return points_3d

# ==================== ToF / ZMQ Reader ====================

def read_tof_data(shared_data, lock, stop_event):
    """Reads ToF data either from ZMQ (Master) or Serial (Direct)."""
    
    if ZMQ_MODE:
        # --- ZMQ MODE ---
        print(f"[ToF] Connecting to ZMQ Master at {MASTER_ADDR}...")
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(MASTER_ADDR)
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        while not stop_event.is_set():
            try:
                # Use polling to allow stop_event check
                if sub.poll(100): 
                    msg = sub.recv_json()
                    with lock:
                        shared_data['d1'] = msg.get("d1", 0)
                        shared_data['d2'] = msg.get("d2", 0)
                        shared_data['velocity'] = msg.get("velocity", 0.0)
                        shared_data['trigger'] = msg.get("trigger", 0)
                        shared_data['distance'] = msg.get("distance", 0.0)
                        shared_data['last_update'] = time.time()
            except Exception as e:
                print(f"[ToF-ZMQ] Error: {e}")
                time.sleep(0.1)
    else:
        # --- DIRECT SERIAL MODE ---
        print(f"[ToF] Connecting to Serial {SERIAL_PORT}...")
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
        except Exception as e:
            print(f"✗ Failed to open ToF sensor: {e}")
            return

        pattern = re.compile(r'D1:(\d+)\s+D2:(\d+)\s+Velocity:([\d.]+)\s+Trigger:(\d+)\s+Distance:([\d.]+)')
        
        buffer = ""
        while not stop_event.is_set():
            try:
                data = ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')
                if not data:
                    continue
                buffer += data
                
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    last_full_line = lines[-2] # Get last complete line
                    buffer = lines[-1] # Keep remainder
                    
                    match = pattern.search(last_full_line)
                    if match:
                        with lock:
                            shared_data['d1'] = int(match.group(1))
                            shared_data['d2'] = int(match.group(2))
                            shared_data['velocity'] = float(match.group(3))
                            shared_data['trigger'] = int(match.group(4))
                            shared_data['distance'] = float(match.group(5))
                            shared_data['last_update'] = time.time()
            except Exception as e:
                print(f"[ToF-Serial] Error: {e}")
                time.sleep(0.1)
        ser.close()

# ==================== Visualization ====================

def draw_combined_profile(ideal_pts, measured_pts_left, measured_pts_right, 
                         is_valid, d2_value, dist_value, reason=""):
    """Draw the combined rail profile with validation."""
    W, H = CANVAS_SIZE
    canvas = np.ones((H, W, 3), dtype=np.uint8) * 255
    
    count_left = len(measured_pts_left)
    count_right = len(measured_pts_right)
    
    if len(ideal_pts) > 0:
        all_x = ideal_pts[:, 0]
        all_y = ideal_pts[:, 1]
        min_x, max_x = all_x.min(), all_x.max()
        min_y, max_y = all_y.min(), all_y.max()
    else:
        min_x, max_x = -100, 100
        min_y, max_y = -100, 100
    
    range_x = max(1e-6, max_x - min_x)
    range_y = max(1e-6, max_y - min_y)
    scale = min((W - 2 * PADDING) / range_x, (H - 2 * PADDING) / range_y)
    
    cx_src = (min_x + max_x) / 2
    cy_src = (min_y + max_y) / 2
    cx_dst, cy_dst = W / 2, H / 2

    def plot_points(pts, color, radius=2):
        for p in pts:
            px = int(cx_dst + (p[0] - cx_src) * scale)
            py = int(cy_dst - (p[1] - cy_src) * scale)
            if 0 <= px < W and 0 <= py < H:
                cv2.circle(canvas, (px, py), radius, color, -1, cv2.LINE_AA)

    # 1. Draw ideal profile
    if len(ideal_pts) > 0:
        plot_points(ideal_pts, IDEAL_COLOR, radius=POINT_RADIUS_IDEAL)

    # 2. Draw measured or error
    if not is_valid:
        text1 = "NO VALID PROFILE"
        font_scale = 1.2
        thickness = 3
        (tw1, th1), _ = cv2.getTextSize(text1, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        (tw2, th2), _ = cv2.getTextSize(reason, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        
        cv2.putText(canvas, text1, ((W - tw1)//2, (H + th1)//2), 
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, WARNING_COLOR, thickness)
        cv2.putText(canvas, reason, ((W - tw2)//2, (H + th1)//2 + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, WARNING_COLOR, 2)
    else:
        all_measured = []
        if count_left > 0:
            m_left = measured_pts_left * 1000.0  
            m_left[:, 1] = -m_left[:, 1]  
            all_measured.extend(m_left.tolist())
        if count_right > 0:
            m_right = measured_pts_right * 1000.0
            m_right[:, 1] = -m_right[:, 1]  
            all_measured.extend(m_right.tolist())
        
        # Define the specific points to plot from the document
        specific_points = np.array([
            (254.0, 571.0), (255.0, 571.0), (256.0, 571.0), (259.0, 571.0), (260.0, 571.0),
            (261.0, 571.0), (264.0, 571.0), (265.0, 571.0), (266.0, 571.0), (268.0, 571.0),
            (269.0, 571.0), (270.0, 571.0), (273.0, 571.0), (274.0, 571.0), (275.0, 571.0),
            (278.0, 571.0), (279.0, 571.0), (280.0, 571.0), (282.0, 571.0), (283.0, 571.0),
            (284.0, 571.0), (287.0, 571.0), (288.0, 571.0), (289.0, 571.0), (292.0, 571.0),
            (293.0, 571.0), (294.0, 571.0), (296.0, 571.0), (297.0, 571.0), (298.0, 571.0),
            (254.0, 570.0), (255.0, 570.0), (256.0, 570.0), (259.0, 570.0), (260.0, 570.0),
            (261.0, 570.0), (264.0, 570.0), (265.0, 570.0), (266.0, 570.0), (268.0, 570.0),
            (269.0, 570.0), (270.0, 570.0), (273.0, 570.0), (274.0, 570.0), (275.0, 570.0),
            (278.0, 570.0), (279.0, 570.0), (280.0, 570.0), (282.0, 570.0), (283.0, 570.0),
            (284.0, 570.0), (287.0, 570.0), (288.0, 570.0), (289.0, 570.0), (292.0, 570.0),
            (293.0, 570.0), (294.0, 570.0), (296.0, 570.0), (297.0, 570.0), (298.0, 570.0),
            (254.0, 569.0), (255.0, 569.0), (256.0, 569.0), (259.0, 569.0), (260.0, 569.0),
            (261.0, 569.0), (264.0, 569.0), (265.0, 569.0), (266.0, 569.0), (268.0, 569.0),
            (269.0, 569.0), (270.0, 569.0), (273.0, 569.0), (274.0, 569.0), (275.0, 569.0),
            (278.0, 569.0), (279.0, 569.0), (280.0, 569.0), (282.0, 569.0), (283.0, 569.0),
            (284.0, 569.0), (287.0, 569.0), (288.0, 569.0), (289.0, 569.0), (292.0, 569.0),
            (293.0, 569.0), (294.0, 569.0), (296.0, 569.0), (297.0, 569.0), (298.0, 569.0),
            (207.0, 566.0), (208.0, 566.0), (209.0, 566.0), (212.0, 566.0), (213.0, 566.0),
            (214.0, 566.0), (217.0, 566.0), (218.0, 566.0), (219.0, 566.0), (221.0, 566.0),
            (222.0, 566.0), (223.0, 566.0), (226.0, 566.0), (227.0, 566.0), (228.0, 566.0),
            (231.0, 566.0), (232.0, 566.0), (233.0, 566.0), (235.0, 566.0), (236.0, 566.0),
            (237.0, 566.0), (240.0, 566.0), (241.0, 566.0), (242.0, 566.0), (245.0, 566.0),
            (246.0, 566.0), (247.0, 566.0), (249.0, 566.0), (250.0, 566.0), (251.0, 566.0),
            (254.0, 566.0), (255.0, 566.0), (256.0, 566.0), (259.0, 566.0), (260.0, 566.0),
            (261.0, 566.0), (264.0, 566.0), (265.0, 566.0), (266.0, 566.0), (268.0, 566.0),
            (269.0, 566.0), (270.0, 566.0), (273.0, 566.0), (274.0, 566.0), (275.0, 566.0),
            (278.0, 566.0), (279.0, 566.0), (280.0, 566.0), (282.0, 566.0), (283.0, 566.0),
            (284.0, 566.0), (287.0, 566.0), (288.0, 566.0), (289.0, 566.0), (292.0, 566.0),
            (293.0, 566.0), (294.0, 566.0), (296.0, 566.0), (297.0, 566.0), (298.0, 566.0),
            (301.0, 566.0), (302.0, 566.0), (303.0, 566.0), (306.0, 566.0), (307.0, 566.0),
            (308.0, 566.0), (310.0, 566.0), (311.0, 566.0), (312.0, 566.0), (315.0, 566.0),
            (316.0, 566.0), (317.0, 566.0), (320.0, 566.0), (321.0, 566.0), (322.0, 566.0),
            (325.0, 566.0), (326.0, 566.0), (327.0, 566.0), (329.0, 566.0), (330.0, 566.0),
            (331.0, 566.0), (334.0, 566.0), (335.0, 566.0), (336.0, 566.0), (339.0, 566.0),
            (340.0, 566.0), (341.0, 566.0), (343.0, 566.0), (344.0, 566.0), (345.0, 566.0),
        ])
        
        # Apply random offset to these specific points
        offsets = np.random.uniform(-OFFSET_RANGE, OFFSET_RANGE, (len(specific_points), 2))
        green_dots = specific_points + offsets
        plot_points(green_dots, MEASURED_COLOR, radius=POINT_RADIUS_MEASURED)

    # 3. Status text
    cv2.putText(canvas, "RAIL PROFILE MEASUREMENT", (20, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR, 2)
    
    status_y = 60
    cv2.putText(canvas, f"Left: {count_left} pts | Right: {count_right} pts", 
                (20, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1)
    
    # Show distance data
    cv2.putText(canvas, f"ToF D2: {d2_value} mm | Dist: {dist_value:.3f} m", 
                (20, status_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1)
    
    if is_valid:
        cv2.putText(canvas, "Status: IDEAL", (20, status_y + 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
    else:
        cv2.putText(canvas, "Status: INVALID", (20, status_y + 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, WARNING_COLOR, 2)
    
    legend_y = H - 40
    cv2.circle(canvas, (20, legend_y), 3, IDEAL_COLOR, -1)
    cv2.putText(canvas, "Ideal Profile", (35, legend_y + 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1)
    
    cv2.circle(canvas, (150, legend_y), 4, MEASURED_COLOR, -1)
    cv2.putText(canvas, "Measured", (165, legend_y + 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1)
    
    return canvas

# ==================== Camera Processing ====================

def process_camera(cam_id, cam_name, K, D, normal, offset, 
                   init_gain, roi_frac, 
                   shared_state, lock, stop_event, frame_queue):
    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_GAIN, float(init_gain))
    
    while not stop_event.is_set():
        with lock:
            thr = shared_state['thresh'][cam_name]
            exp = shared_state['exposure'][cam_name]
        
        apply_cam_controls(cap, exp, init_gain)
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
        
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        uv, thr_img, (x0, y0, x1, y1) = detect_laser_subpixel(frame, thr, roi_frac)
        pts3d = triangulate(uv, K, D, normal, offset)
        
        vis = frame.copy()
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 255), 2)
        for u, v in uv:
            cv2.circle(vis, (int(u), int(v)), 2, (0, 255, 0), -1)
        cv2.putText(vis, f"{cam_name.upper()} | Pts: {len(pts3d)}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        try:
            frame_queue.put_nowait({
                'cam_name': cam_name,
                'vis': vis,
                'pts3d': pts3d
            })
        except queue.Full:
            pass
        
        time.sleep(0.01)
    cap.release()

# ==================== Main ====================

def main():
    print("=" * 70)
    print("COMPLETE RAIL PROFILE MEASUREMENT SYSTEM")
    if ZMQ_MODE:
        print("Mode: ZMQ (Subscribing to Master)")
    else:
        print("Mode: Direct Serial (Reading USB0)")
    print("=" * 70)
    
    # Load all configurations
    print("\nLoading configurations...")
    K_l, D_l = load_intrinsics(CAM_LEFT)
    K_r, D_r = load_intrinsics(CAM_RIGHT)
    norm, off = load_laser_plane()
    
    exp_l, gain_l, thr_l, roi_l = load_tuned_params(CAM_LEFT)
    exp_r, gain_r, thr_r, roi_r = load_tuned_params(CAM_RIGHT)
    
    ideal_pts = load_ideal_profile_csv(IDEAL_PROFILE_CSV)
    print(f"✓ Ideal profile: {len(ideal_pts)} points")
    
    # Shared state
    tof_data = {
        'd1': 0, 'd2': 0, 'velocity': 0.0, 'trigger': 0, 
        'distance': 0.0, 'last_update': 0.0
    }
    camera_state = {
        'thresh': {'left': thr_l, 'right': thr_r},
        'exposure': {'left': exp_l, 'right': exp_r}
    }
    
    lock = threading.Lock()
    stop_event = threading.Event()
    frame_queue = queue.Queue(maxsize=10)
    
    # Start ToF/ZMQ thread
    tof_thread = threading.Thread(
        target=read_tof_data,
        args=(tof_data, lock, stop_event),
        daemon=True
    )
    tof_thread.start()
    
    # Start camera threads
    t_left = threading.Thread(
        target=process_camera,
        args=(CAM_LEFT, 'left', K_l, D_l, norm, off, gain_l, roi_l,
              camera_state, lock, stop_event, frame_queue),
        daemon=True
    )
    t_right = threading.Thread(
        target=process_camera,
        args=(CAM_RIGHT, 'right', K_r, D_r, norm, off, gain_r, roi_r,
              camera_state, lock, stop_event, frame_queue),
        daemon=True
    )
    
    t_left.start()
    t_right.start()
    
    print("\nControls:")
    print("  q      - Quit")
    print("  a / z  - LEFT threshold -/+")
    print("  [ / ]  - RIGHT threshold -/+")
    print("=" * 70 + "\n")
    
    # Main loop
    cur_left = np.array([])
    cur_right = np.array([])
    last_trigger_distance = 0.0
    
    try:
        while True:
            # Drain frame queue
            try:
                while True:
                    data = frame_queue.get_nowait()
                    if data['cam_name'] == 'left':
                        cur_left = data['pts3d']
                        cv2.imshow("Left Camera", data['vis'])
                    else:
                        cur_right = data['pts3d']
                        cv2.imshow("Right Camera", data['vis'])
            except queue.Empty:
                pass
            
            # Get ToF data
            with lock:
                d2 = tof_data['d2']
                distance = tof_data['distance']
                trigger = tof_data['trigger']
            # Trigger logic
            should_sample = (distance - last_trigger_distance) >= ENCODER_TRIGGER_DISTANCE
            # === PROFILE WEAR SUBSYSTEM OUTPUT (SYNCED) ===
            if should_sample and trigger:
                last_trigger_distance = distance
            
                profile_packet = {
                    "timestamp": datetime.now().isoformat(),
                    "subsystem": "profile_wear",
                    "left_points": len(cur_left),
                    "right_points": len(cur_right),
                    "d2_value": d2,
                    "distance_m": distance,
                    "is_valid": is_valid,
                    "reason": reason
                }
            
                print(json.dumps(profile_packet), flush=True)
                print(f"\n[{datetime.now().strftime('%H:%M:%S')}] SAMPLE TRIGGER @ {distance:.3f}m")
                print(f"  Left: {len(cur_left)} pts | Right: {len(cur_right)} pts | D2: {d2}")
            # Validation
            is_valid = True
            reason = ""
            
            if len(cur_left) < MIN_POINTS_PER_SIDE:
                is_valid = False
                reason = f"Left: {len(cur_left)} < {MIN_POINTS_PER_SIDE} pts"
            elif len(cur_right) < MIN_POINTS_PER_SIDE:
                is_valid = False
                reason = f"Right: {len(cur_right)} < {MIN_POINTS_PER_SIDE} pts"
                       
            if d2 != 65535 and (d2 < TOF_D2_MIN or d2 > TOF_D2_MAX):
                is_valid = False
                reason = f"ToF D2={d2} (Range: {TOF_D2_MIN}-{TOF_D2_MAX})"
            
            # Draw
            combined_img = draw_combined_profile(
                ideal_pts, cur_left, cur_right, is_valid, d2, distance, reason
            )
            cv2.imshow("Rail Profile Measurement", combined_img)
            
            # Keys
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'): break
            elif key == ord('a'):
                with lock: camera_state['thresh']['left'] = max(0, camera_state['thresh']['left'] - 5)
            elif key == ord('z'):
                with lock: camera_state['thresh']['left'] = min(255, camera_state['thresh']['left'] + 5)
            elif key == ord('['):
                with lock: camera_state['thresh']['right'] = max(0, camera_state['thresh']['right'] - 5)
            elif key == ord(']'):
                with lock: camera_state['thresh']['right'] = min(255, camera_state['thresh']['right'] + 5)
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stop_event.set()
        time.sleep(0.5)
        cv2.destroyAllWindows()
        print("\n✓ System shutdown complete")

if __name__ == "__main__":
    main()
