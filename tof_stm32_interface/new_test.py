import math
import os
import time
import threading
import serial
import re
import csv
import sys
import json

# --- CONFIGURATION ---
PIPE_PATH = "/tmp/xsens_pipe"
CSV_FILENAME = "track_data_log.csv"

# Angles in RADIANS
angle1 = math.radians(30)
angle2 = math.radians(60)

# ===============================
# 1. LOGGING & OUTPUT UTILITIES
# ===============================

class CSVLogger:
    def __init__(self, filename):
        self.filename = filename
        # Check if file exists to decide whether to write headers
        file_exists = os.path.isfile(self.filename)
        
        self.file = open(self.filename, 'a', newline='')
        self.writer = csv.writer(self.file)
        
        if not file_exists:
            # Write Header
            self.writer.writerow([
                "timestamp", "velocity", 
                "dist_1", "dist_2", "dist_3", "dist_4",
                "acc_y", "acc_z", 
                "gauge_width", "cross_level", "twist_spatial", "trigger"
            ])
            self.file.flush()

    def log(self, timestamp, raw_tof, raw_imu, results):
        row = [
            f"{timestamp:.3f}",
            f"{results['velocity']:.3f}",
            raw_tof[0], raw_tof[1], raw_tof[2], raw_tof[3],
            f"{raw_imu['ay']:.3f}", f"{raw_imu['az']:.3f}",
            f"{results['gauge']:.2f}",
            f"{results['cross']:.2f}",
            f"{results['twist']:.2f}",
            raw_tof[5] # Trigger
        ]
        self.writer.writerow(row)
        self.file.flush() # Ensure data is written immediately

    def close(self):
        self.file.close()

def log_to_stderr(message):
    """Prints to console even if stdout is redirected to a pipe."""
    sys.stderr.write(f"{message}\n")
    sys.stderr.flush()

def send_to_pipe(data_dict):
    """
    Writes clean JSON to stdout for the Bash pipe.
    Bash usage: python data.py > /tmp/track_pipe
    """
    json_str = json.dumps(data_dict)
    sys.stdout.write(json_str + "\n")
    sys.stdout.flush() # CRITICAL: Flushes buffer to pipe immediately

# ===============================
# 2. SENSOR CLASSES (Existing Logic)
# ===============================

class IMUListener(threading.Thread):
    def __init__(self, pipe_path):
        super().__init__()
        self.pipe_path = pipe_path
        self.daemon = True
        self.running = True
        self.lock = threading.Lock()
        self.data = {'ax': 0.0, 'ay': 0.0, 'az': 0.0, 'qw': 1.0}

    def run(self):
        while not os.path.exists(self.pipe_path):
            log_to_stderr(f"Waiting for IMU pipe {self.pipe_path}...")
            time.sleep(1)

        log_to_stderr(f"IMU: Connected to {self.pipe_path}")

        try:
            with open(self.pipe_path, 'r') as fifo:
                while self.running:
                    line = fifo.readline().strip()
                    if not line:
                        time.sleep(0.005)
                        continue
                    self._parse(line)
        except Exception as e:
            log_to_stderr(f"IMU Pipe Error: {e}")

    def _parse(self, line):
        parts = line.split()
        temp = {}
        try:
            for i, tag in enumerate(parts):
                if tag == "ACC:" and i+3 < len(parts):
                    temp['ay'] = float(parts[i+2])
                    temp['az'] = float(parts[i+3])
            with self.lock:
                self.data.update(temp)
        except ValueError:
            pass

    def get_data(self):
        with self.lock:
            return self.data.copy()

class ToFArrayReader:
    def __init__(self, port="/dev/ttyTHS1", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        # d1, d2, d3, d4, vel, trig
        self.data = [0, 0, 0, 0, 0.0, 0]
        
        # Regex for robustness
        self.pattern_dist = re.compile(r"D\s*(\d)\s*:\s*(\d+)")
        self.pattern_vel = re.compile(r"Velocity\s*:\s*([\d\.]+)")
        self.pattern_trig = re.compile(r"Triger\s*:\s*(\d+)")

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            log_to_stderr("UART: Connected.")
        except Exception as e:
            log_to_stderr(f"UART Error: {e}")

    def read(self):
        if not self.ser: return self.data
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "D" in line: self._parse(line)
        except: pass
        return self.data

    def _parse(self, line):
        matches = self.pattern_dist.findall(line)
        for sid, val in matches:
            idx = int(sid) - 1
            if 0 <= idx < 4: self.data[idx] = int(val)
        
        v_match = self.pattern_vel.search(line)
        if v_match: self.data[4] = float(v_match.group(1))

        t_match = self.pattern_trig.search(line)
        if t_match: self.data[5] = int(t_match.group(1))

# ===============================
# 3. MATH
# ===============================

def safety_calculation(imu, tof_data, prev_cl, prev_time):
    d1, d2, d3, d4 = tof_data[0:4]
    velocity = tof_data[4]
    ay, az = imu['ay'], imu['az']

    # 1. Gauge
    gauge = (d1 * math.sin(angle1)) + (d2 * math.sin(angle2))

    # 2. Cross Level
    tilt = math.atan2(ay, az) if az != 0 else 0.0
    cross = math.sin(tilt) * gauge

    # 3. Twist (Spatial)
    now = time.time()
    twist = 0.0
    
    if prev_time is not None:
        dt = max(1e-4, now - prev_time)
        delta_cl = cross - prev_cl
        
        # Calculate twist if moving
        if abs(velocity) > 0.1:
            dist = velocity * dt
            twist = delta_cl / dist # mm/m
    
    res = {
        "gauge": gauge, "cross": cross, "twist": twist, 
        "velocity": velocity, "timestamp": now
    }
    return res, cross, now

# ===============================
# 4. MAIN EXECUTION
# ===============================

if __name__ == "__main__":
    # Start Sensors
    imu_thread = IMUListener(PIPE_PATH)
    imu_thread.start()

    tof_reader = ToFArrayReader()
    tof_reader.connect()

    # Start CSV Logger
    csv_log = CSVLogger(CSV_FILENAME)
    
    log_to_stderr("System Ready. Writing data to stdout (Pipe) and CSV...")

    prev_cl = 0.0
    prev_time = time.time()

    try:
        while True:
            # 1. Read Sensors
            imu_data = imu_thread.get_data()
            tof_data = tof_reader.read()

            # 2. Calculate
            result, prev_cl, prev_time = safety_calculation(
                imu_data, tof_data, prev_cl, prev_time
            )

            # 3. Log to CSV (The "Stack" on disk)
            csv_log.log(result['timestamp'], tof_data, imu_data, result)

            # 4. Send to Pipe (Standard Output)
            # This is what goes to /tmp/track_pipe
            send_to_pipe(result)

            # 5. Visual Status (Optional, goes to stderr so it doesn't pollute pipe)
            # log_to_stderr(f"Status: V={result['velocity']:.1f} Twist={result['twist']:.2f}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        log_to_stderr("\nStopping...")
        csv_log.close()
        imu_thread.running = False
