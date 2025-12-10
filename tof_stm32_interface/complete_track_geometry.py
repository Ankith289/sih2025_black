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
# 1. PURE CALCULATION FUNCTION
# ============================

def calculate_track_parameters(d1, d2, velocity, ay, az, prev_cross, prev_time):
    """
    Performs all physics and geometry calculations.
    Returns: (results_dict, updated_cross_level, updated_timestamp)
    """
    now = time.time()
    
    # --- A. GAUGE WIDTH ---
    # Formula: d1*sin(30) + d2*sin(60)
    gauge = (d1 * math.sin(ANGLE1)) + (d2 * math.sin(ANGLE2))

    # --- B. CROSS LEVEL (CANT) ---
    # Calculate tilt angle from accelerometer (ay, az)
    # atan2 handles the division by zero if az is 0
    down= math.sqrt(pow(abs(ay),2)+pow(abs(az),2))
    if 9 < down and down < 10 and abs(az)<9:
        cross_level_angle = math.degrees(math.asin(az/down))
        cross_level= math.sin(math.radians(cross_level_angle))*gauge
    else:
        cross_level=0
    # --- C. TWIST (Spatial) ---
    # Twist = (Change in Cross Level) / (Distance Traveled)
    twist = 0.0
    
    if prev_time is not None:
        dt = now - prev_time
        
        # Only calculate twist if moving > 0.5 m/s
        if dt > 0 and abs(velocity) > 0.5:
            dist_traveled = velocity * dt
            
            # Prevent division by tiny distances (noise)
            if dist_traveled > 0.001:
                twist = (cross_level - prev_cross) / dist_traveled

    # Pack results
    results = {
        "ts": round(now, 3),
        "vel": round(velocity, 2),
        "gauge": round(gauge, 2),
        "cross": round(cross_level, 2),
        "twist": round(twist, 2),
        "d1": int(d1),
        "d2": int(d2)
    }

    return results, cross_level, now


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
            "d1": 0, "d2": 0, "velocity": 0.0, "trigger": 0,
            "ay": 0.0, "az": 9.8  # Default gravity
        }

    def update_imu(self, ay, az):
        with self.lock:
            self.data["ay"] = ay
            self.data["az"] = az

    def update_uart(self, d1, d2, vel, trig):
        with self.lock:
            self.data["d1"] = d1
            self.data["d2"] = d2
            self.data["velocity"] = vel
            self.data["trigger"] = trig

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
    except: return

    regex = re.compile(r"(D1|D2|Velocity|Triger)\s*:\s*([\d\.]+)")
    
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue
            matches = regex.findall(line)
            if matches:
                # Create a temporary dict to update all fields at once
                updates = {}
                for k, v in matches: updates[k] = float(v)
                
                # Only update if we got valid data
                if 'D1' in updates:
                    state.update_uart(
                        int(updates.get('D1', 0)),
                        int(updates.get('D2', 0)),
                        updates.get('Velocity', 0.0),
                        int(updates.get('Triger', 0))
                    )
        except: pass


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
            results, prev_cross, prev_time = calculate_track_parameters(
                d1=inputs['d1'],
                d2=inputs['d2'],
                velocity=inputs['velocity'],
                ay=inputs['ay'],
                az=inputs['az'],
                prev_cross=prev_cross,
                prev_time=prev_time
            )
            results["unevenness"] = round(vertical_unevenness, 3)
            results["ride_index"] = round(ride_rms, 3)
            # 3. Output to Pipe (Web)
            print(json.dumps(results), flush=True)

            # 4. Output to CSV (Disk)
            writer.writerow([
                results['ts'], results['vel'], results['gauge'], 
                results['cross'], results['twist'], 
                results['d1'], results['d2'], 
                inputs['ay'], inputs['az'],
                results["unevenness"],
                results["ride_index"]
            ])
            log_file.flush()

            # 5. Maintain Refresh Rate
            elapsed = time.time() - loop_start
            time.sleep(max(0, (1.0/WEB_REFRESH_RATE) - elapsed))

    except KeyboardInterrupt:
        log_file.close()

if __name__ == "__main__":
    main()
