import math
import os
import time
import threading
import struct
import serial

# --- CONFIGURATION ---
PIPE_PATH = "/tmp/xsens_pipe"

angle1 = math.degrees(30)
angle2 = math.degrees(60)

previous_cross_level = 0.0

class IMUListener(threading.Thread):
    """
    Background thread that reads the text pipe from the C++ Driver.
    Does not block the main ToF loop.
    """
    def __init__(self, pipe_path):
        super().__init__()
        self.pipe_path = pipe_path
        self.daemon = True
        self.running = True
        self.data = {
            'ax': 0.0, 'ay': 0.0, 'az': 0.0,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
            'qw': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0
        }

    def run(self):
        # Wait for C++ driver to create the pipe
        while not os.path.exists(self.pipe_path):
            time.sleep(0.5)

        print(f"IMU: Connected to {self.pipe_path}")
        try:
            with open(self.pipe_path, 'r') as fifo:
                while self.running:
                    line = fifo.readline()
                    if not line:
                        time.sleep(0.01)
                        continue
                    self._parse(line)
        except Exception as e:
            print(f"IMU Pipe Error: {e}")

    def _parse(self, line):
        """Parses: QUAT: ... ACC: ... GYRO: ..."""
        parts = line.split()
        try:
            # We iterate through the line looking for keywords
            for i, tag in enumerate(parts):
                if tag == "ACC:":
                    self.data['ax'] = float(parts[i+1])
                    self.data['ay'] = float(parts[i+2])
                    self.data['az'] = float(parts[i+3])
                elif tag == "GYRO:":
                    self.data['gx'] = float(parts[i+1])
                    self.data['gy'] = float(parts[i+2])
                    self.data['gz'] = float(parts[i+3])
                elif tag == "QUAT:":
                    # If you need orientation
                    self.data['qw'] = float(parts[i+1])
        except (IndexError, ValueError):
            pass

    def get_data(self):
        return self.data

class ToFArrayReader:
    """
    Reads 4 distances (8 bytes) via UART from Blue Pill.
    """
    def __init__(self, port="/dev/ttyTHS1", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.distances = [0,0,0,0]

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print("UART: Connected to Blue Pill.")
        except Exception as e:
            print(f"UART Connection Error: {e}")

    def read(self):
        if not self.ser:
            return self.distances

        try:
            # Read exactly 8 bytes (d1,d2,d3,d4)
            data = self.ser.read(8)
            if len(data) == 8:
                # Little-endian unsigned short  (matching Blue Pill)
                self.distances = list(struct.unpack("<HHHH", data))
        except:
            pass

        return self.distances

import math
import time

# Tunable constants (move to config)
ANGLE1_DEG = 30.0
ANGLE2_DEG = 60.0
ANGLE1 = math.radians(ANGLE1_DEG)
ANGLE2 = math.radians(ANGLE2_DEG)
GRAVITY = 9.80665           # m/s^2
GRAVITY_TOLERANCE = 0.25    # acceptable deviation (m/s^2)
ACC_Y_MAX = 9.0             # used in your original logic (but we'll use more robust test)
MIN_VALID_TOF = 1           # mm
MAX_VALID_TOF = 20000       # mm
EMA_ALPHA = 0.3             # smoothing for ToF and cross_level

def clamp(x, a, b):
    return max(a, min(b, x))

def safety_calculation(imu, tofs, previous_cross_level, prev_time=None):
    """
    imu: dict with 'ax','ay','az','gx','gy','gz', optionally quaternions
    tofs: list/iterable of 4 distance values (units: mm)
    previous_cross_level: previous cross_level (units: mm)
    prev_time: previous timestamp (seconds); if None, dt will be approximated as 0.05s

    Returns: (decision_dict, updated_previous_cross_level, timestamp)
    """
    now = time.time()
    if prev_time is None:
        dt = 0.05
    else:
        dt = max(1e-6, now - prev_time)

    # --- sanitize ToF readings ---
    d = []
    for v in tofs:
        try:
            vv = int(v)
        except Exception:
            vv = 0
        # reject obviously invalid readings
        if vv < MIN_VALID_TOF or vv > MAX_VALID_TOF:
            vv = 0
        d.append(vv)
    d1, d2, d3, d4 = d

    # If too many invalids, return a safe default
    valid_count = sum(1 for x in d if x > 0)
    if valid_count < 2:
        # not enough sensors online
        decision = {"gauge_width": 0.0, "cross_level": previous_cross_level,
                    "twist_level": 0.0, "previous_cross_level": previous_cross_level,
                    "curve_level": 0.0, "status": "TOF_FAIL"}
        return decision, previous_cross_level, now

    # --- accelerometer magnitude ---
    ax = float(imu.get('ax', 0.0))
    ay = float(imu.get('ay', 0.0))
    az = float(imu.get('az', 0.0))
    down = math.sqrt(ax*ax + ay*ay + az*az)  # total accel magnitude

    # Is sensor near gravity? (simple stationary check)
    near_gravity = abs(down - GRAVITY) < GRAVITY_TOLERANCE + 0.2  # allow small extra slack

    # compute gauge_width (in mm). convert trig inputs to use radians angles
    gauge_width = 0.0
    try:
        gauge_width = (d1 * math.sin(ANGLE1)) + (d2 * math.sin(ANGLE2))
    except Exception:
        gauge_width = 0.0

    # compute cross_level using ax/ay. If IMU oriented differently, replace with corrected axis.
    cross_level = 0.0
    if near_gravity and abs(ay) < GRAVITY + 1.0:
        # derive small angle using asin, ensure argument safe
        denom = max(1e-6, down)
        asin_arg = clamp(ay / denom, -1.0, 1.0)
        cross_level_angle = math.degrees(math.asin(asin_arg))
        cross_level = math.sin(math.radians(cross_level_angle)) * gauge_width
    else:
        # fallback — set cross_level to previous filtered value
        cross_level = previous_cross_level

    # apply simple exponential filter to cross_level to smooth noise
    filtered_cross = EMA_ALPHA * cross_level + (1.0 - EMA_ALPHA) * previous_cross_level

    # twist level: rate of change (mm / s)
    twist_level = (filtered_cross - previous_cross_level) / dt

    # curve_level placeholder (could use yaw rate from IMU gx/gy/gz or ToF asymmetry)
    curve_level = 0.0

    decision = {
        "gauge_width": gauge_width,
        "cross_level": filtered_cross,
        "twist_level": twist_level,
        "previous_cross_level": filtered_cross,
        "curve_level": curve_level,
        "status": "OK"
    }

    return decision, filtered_cross, now

# --- MAIN LOOP ---
if __name__ == "__main__":
    # 1. Start IMU Thread (Reads the Pipe)
    imu_thread = IMUListener(PIPE_PATH)
    imu_thread.start()

    # 2. Setup I2C (Reads the Blue Pill)
    tof_array = ToFArrayReader("/dev/ttyTHS1", 115200)
    tof_array.connect()

    print("Fusion System Started...")
prev_time = None
    try:
        while True:
            # A. Get Synchronized Data
            imu_data = imu_thread.get_data()
            tof_data = tof_array.read() # Returns [d1, d2, d3, d4]
            
            # B. Run Logic
            decision, previous_cross_level, prev_time = safety_calculation(imu_data, tof_data, previous_cross_level, prev_time)
            # C. Print
            # Using f-strings to format nicely
            print(f"TOF: {tof_data} | "
                  f"ACC: {imu_data['ax']:.2f} | "
                  f"STATUS: {status}", end='\r')

            time.sleep(0.05) # 20Hz

    except KeyboardInterrupt:
        print("\nStopping...")
        imu_thread.running = False

