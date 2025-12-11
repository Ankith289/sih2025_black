import serial
import threading
import time
import math
import json
import sys
import re
import csv
import os
import signal
import copy
import collections
import statistics

# ============================
# CONFIGURATION
# ============================
IMU_PIPE_PATH = "/tmp/xsens_pipe"
UART_PORT = "/dev/ttyTHS1"
BAUD_RATE = 115200
CSV_FILE_PATH = "track_data_log.csv"
WEB_REFRESH_RATE = 30  # Hz

# Track Geometry Constants (Radians)
ANGLE1 = math.radians(30)
ANGLE2 = math.radians(60)

# ============================
# GLOBAL TIMESTAMP READER (SAFE)
# ============================
TS_FILE = "/tmp/global_timestamp.txt"
_last_ts = "NO_TIMESTAMP"
_last_read = 0

def get_global_timestamp():
    """
    Reads shared timestamp at most every 5ms.
    Prevents heavy I/O across 6 subsystems.
    """
    global _last_ts, _last_read
    now = time.time()

    if now - _last_read >= 0.005:  # Read file max 200 Hz
        try:
            with open(TS_FILE, "r") as f:
                _last_ts = f.read().strip()
        except:
            _last_ts = "NO_TIMESTAMP"
        _last_read = now

    return _last_ts

# ============================
# 1. PURE CALCULATION FUNCTION
# ============================

def calculate_track_parameters(d1, d2, velocity, ay, az, prev_cross, prev_chain):
    """
    Performs track geometry calculations using TF-Minis + IMU + chainage.
    """

    # Handle invalid TF-Mini data
    if d2 == 65535:
        d2 = 0

    # Gauge
    gauge = (d1 * math.sin(ANGLE1)) + (d2 * math.sin(ANGLE2))

    # Cross level: tilt from accelerometer
    down = math.sqrt(ay*ay + az*az)
    if 9.0 < down < 10.1 and abs(az) < 9.0:
        angle = math.asin(az / down)
        cross_level = math.sin(angle) * gauge
    else:
        cross_level = 0

    # Twist from chainage difference
    twist = 0.0
    if prev_chain is not None and abs(chainage - prev_chain) > 0.001:
        delta = chainage - prev_chain
        twist = (cross_level - prev_cross) / delta

    results = {
        "ts": round(time.time(), 3),
        "vel": round(velocity, 2),
        "gauge": round(gauge, 2),
        "cross": round(cross_level, 2),
        "twist": round(twist, 4),
        "d1": int(d1),
        "d2": int(d2),
        "chainage": chainage
    }

    return results, cross_level, chainage

# ============================
# 2. SHARED MEMORY & THREADS
# ============================

class RideQualityEstimator:
    def __init__(self, window_seconds=2.0, sample_rate=30):
        # Store ~2 seconds of data
        self.maxlen = int(window_seconds * sample_rate)
        self.buffer_az = collections.deque(maxlen=self.maxlen)
        self.buffer_ay = collections.deque(maxlen=self.maxlen)

    def update(self, ay, az):
        """Add new sensor readings to history"""
        self.buffer_ay.append(ay)
        self.buffer_az.append(az)

    def calculate_metrics(self):
        """
        Returns:
        - unevenness (Standard Deviation of Vertical Accel)
        - ride_index (RMS of Vertical Accel - Proxy for Comfort)
        """
        if len(self.buffer_az) < 5:
            return 0.0, 0.0 # Not enough data yet

        # 1. Vertical Unevenness (Standard Deviation)
        # Represents roughness/bumping relative to gravity
        try:
            unevenness = statistics.stdev(self.buffer_az)
        except:
            unevenness = 0.0

        # 2. Ride Index (Simplified RMS)
        # RMS = sqrt( mean( x^2 ) )
        # Subtract gravity (9.8) to get dynamic vibration only
        dynamic_accel = [x - 9.81 for x in self.buffer_az]
        sq_sum = sum(x*x for x in dynamic_accel)
        rms_val = math.sqrt(sq_sum / len(self.buffer_az))
        
        # Sperling's Ride Index (Wz) Approximate Heuristic:
        # Wz approx 0.896 * (RMS_Accel)^0.3 (for specific frequencies)
        # We will return raw RMS as it is the standard engineering metric.
        
        return unevenness, rms_val

class SystemState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "d1": 0, "d2": 0,
            "velocity": 0.0,
            "trigger": 0,
            "chainage": 0.0,   # <-- NEW
            "ay": 0.0, "az": 9.8
        }
    def update_imu(self, ay, az):
        with self.lock:
            self.data["ay"] = ay
            self.data["az"] = az

    def update_uart(self, d1, d2, vel, trig, chainage):
        with self.lock:
            self.data["d1"] = d1
            self.data["d2"] = d2
            self.data["velocity"] = vel
            self.data["trigger"] = trig
            self.data["chainage"] = chainage

    def get_snapshot(self):
        with self.lock:
            return copy.deepcopy(self.data)

state = SystemState()

def imu_worker():
    while not os.path.exists(IMU_PIPE_PATH): time.sleep(1)
    sys.stderr.write("IMU: Connected.\n")
    try:
        with open(IMU_PIPE_PATH, 'r') as f:
            while True:
                line = f.readline()
                if not line: continue
                parts = line.split()
                if "ACC:" in parts:
                    try:
                        idx = parts.index("ACC:")
                        state.update_imu(float(parts[idx+2]), float(parts[idx+3]))
                    except: pass
    except: pass


def uart_worker():
    try:
        ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()
        sys.stderr.write(f"UART: Connected ({UART_PORT}).\n")
    except:
        return

    # Now also parses: Distance:<value>
    regex = re.compile(r"(D1|D2|Velocity|Triger|Distance)\s*:\s*([\d\.]+)")
    
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            matches = regex.findall(line)
            if not matches:
                continue

            updates = {}
            for key, val in matches:
                updates[key] = float(val)

            if "D1" in updates:
                state.update_uart(
                    d1=int(updates.get("D1", 0)),
                    d2=int(updates.get("D2", 0)),
                    vel=updates.get("Velocity", 0.0),
                    trig=int(updates.get("Triger", 0)),
                    chainage=updates.get("Distance", 0.0)
                )
        except:
            pass

# ============================
# SYNC PACKET BUILDER
# ============================
def build_sync_packet(results):
    """
    Convert Track Geometry results into a unified packet
    that Code B (synchronizer) understands.
    """
    return {
        "timestamp": get_global_timestamp(),  # string timestamp
        "subsystem": "track_geometry",
        "gauge": results["gauge"],
        "cross_level": results["cross"],
        "twist": results["twist"],
        "unevenness": results["unevenness"],
        "ride_index": results["ride_index"],
        "chainage": results.get("chainage", 0.0),
        "velocity": results.get("vel", 0.0),
        "d1": results["d1"],
        "d2": results["d2"]
    }


# ============================
# 3. MAIN LOOP
# ============================

def main():
    # Start inputs
    threading.Thread(target=imu_worker, daemon=True).start()
    threading.Thread(target=uart_worker, daemon=True).start()

    # Setup Logging
    log_file = open(CSV_FILE_PATH, 'a', newline='')
    writer = csv.writer(log_file)
    if os.stat(CSV_FILE_PATH).st_size == 0:
        writer.writerow(["ts", "vel", "gauge", "cross", "twist", "d1", "d2", "ay", "az"])

    # History vars for twist calculation
    prev_cross = 0.0
    prev_time = time.time()

    sys.stderr.write(f"Starting Output Loop at {WEB_REFRESH_RATE}Hz...\n")

    try:
        # ... existing setup ...
        ride_quality = RideQualityEstimator(window_seconds=1.0, sample_rate=WEB_REFRESH_RATE)
        while True:
            loop_start = time.time()

            # 1. Get Inputs
            inputs = state.get_snapshot()
            ride_quality.update(inputs['ay'], inputs['az'])

            vertical_unevenness, ride_rms = ride_quality.calculate_metrics()
            # 2. CALL THE CALCULATION FUNCTION
            results, prev_cross, prev_chain = calculate_track_parameters(
                d1=inputs['d1'],
                d2=inputs['d2'],
                velocity=inputs['velocity'],
                ay=inputs['ay'],
                az=inputs['az'],
                prev_cross=prev_cross,
                prev_chain=prev_chain
            )
            results["unevenness"] = round(vertical_unevenness, 3)
            results["ride_index"] = round(ride_rms, 3)
            # 3. Output to Pipe (Web)
            sync_packet = build_sync_packet(results)
            # Correct output for Code B
            if inputs["trigger"] == 1:
                print(json.dumps(sync_packet), flush=True)
            # 4. Output to CSV (Disk)
            writer.writerow([
                results['ts'], results['vel'], results['gauge'], 
                results['cross'], results['twist'], 
                results['d1'], results['d2'], 
                inputs['ay'], inputs['az'],
                results["unevenness"],
                results["ride_index"],
                results["chainage"]
            ])
            log_file.flush()

            # 5. Maintain Refresh Rate
            elapsed = time.time() - loop_start
            time.sleep(max(0, (1.0/WEB_REFRESH_RATE) - elapsed))

    except KeyboardInterrupt:
        log_file.close()

if __name__ == "__main__":
    main()
