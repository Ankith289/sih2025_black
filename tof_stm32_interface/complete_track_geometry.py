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

# ============================
# CONFIGURATION
# ============================
IMU_PIPE_PATH = "/tmp/xsens_pipe"
UART_PORT = "/dev/ttyTHS1"
BAUD_RATE = 115200
CSV_FILE_PATH = "track_data_log.csv"
WEB_REFRESH_RATE = 30  # Hz (Output rate to the pipe)

# Track Geometry (Radians)
ANGLE1 = math.radians(30)
ANGLE2 = math.radians(60)

# ============================
# 1. SHARED MEMORY (The "Stack")
# ============================
class SystemState:
    """Thread-safe storage for the latest sensor data."""
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            # UART Data
            "d1": 0, "d2": 0, "velocity": 0.0, "trigger_id": 0,
            # IMU Data
            "ay": 0.0, "az": 9.8,
            # Metadata
            "last_uart_ts": 0.0
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
            self.data["trigger_id"] = trig
            self.data["last_uart_ts"] = time.time()

    def get_snapshot(self):
        with self.lock:
            return copy.deepcopy(self.data)

# Global Instance
state = SystemState()

# ============================
# 2. LOGGING UTILITY
# ============================
class CSVLogger:
    def __init__(self, filename):
        self.filename = filename
        self.file = open(self.filename, 'a', newline='')
        self.writer = csv.writer(self.file)
        
        # Write header if file is empty
        if os.stat(self.filename).st_size == 0:
            self.writer.writerow(["timestamp", "velocity", "gauge", "cross", "twist", "d1", "d2", "ay", "az"])
            self.file.flush()

    def log(self, row_data):
        self.writer.writerow(row_data)
        # Periodic flush (optional, strictly speaking OS handles this but safer for logs)
        self.file.flush() 

    def close(self):
        self.file.close()

# ============================
# 3. WORKER THREADS (Inputs)
# ============================

def imu_worker():
    """Reads High-Speed IMU Data"""
    while not os.path.exists(IMU_PIPE_PATH):
        time.sleep(1)
    
    sys.stderr.write("IMU: Connected.\n")
    
    try:
        with open(IMU_PIPE_PATH, 'r') as fifo:
            while True:
                line = fifo.readline()
                if not line: continue
                
                parts = line.split()
                try:
                    # Quick parse for ACC:
                    if "ACC:" in parts:
                        idx = parts.index("ACC:")
                        # Ensure we have enough parts
                        if idx + 3 < len(parts):
                            ay = float(parts[idx+2])
                            az = float(parts[idx+3])
                            state.update_imu(ay, az)
                except:
                    pass
    except Exception as e:
        sys.stderr.write(f"IMU Worker Failed: {e}\n")

def uart_worker():
    """Reads UART Data (STM32) whenever it arrives"""
    try:
        ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()
        sys.stderr.write(f"UART: Connected on {UART_PORT}.\n")
    except Exception as e:
        sys.stderr.write(f"UART Failed: {e}\n")
        return

    # Compile regex once
    regex = re.compile(r"(D1|D2|Velocity|Triger)\s*:\s*([\d\.]+)")
    
    # Local buffer to hold partial updates
    local_data = {'D1': 0, 'D2': 0, 'Velocity': 0.0, 'Triger': 0}

    while True:
        try:
            # Blocking read (efficient)
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue

            matches = regex.findall(line)
            if matches:
                for key, val in matches:
                    local_data[key] = float(val)
                
                # Update shared state immediately upon receipt
                state.update_uart(
                    int(local_data['D1']), 
                    int(local_data['D2']), 
                    local_data['Velocity'],
                    int(local_data['Triger'])
                )
        except Exception:
            pass

# ============================
# 4. MATH & MAIN LOOP (Output)
# ============================

def calculate_twist(current_cross, prev_cross, velocity, prev_time):
    now = time.time()
    dt = now - prev_time
    twist = 0.0
    
    # Only calculate twist if moving and time has passed
    if dt > 0 and abs(velocity) > 0.5:
        dist = velocity * dt
        if dist > 0.001:
            twist = (current_cross - prev_cross) / dist
            
    return twist, now

def main():
    # Start Threads
    t1 = threading.Thread(target=imu_worker, daemon=True)
    t2 = threading.Thread(target=uart_worker, daemon=True)
    t1.start()
    t2.start()

    logger = CSVLogger(CSV_FILE_PATH)
    
    prev_cross = 0.0
    prev_time = time.time()

    sys.stderr.write(f"System Running. Outputting {WEB_REFRESH_RATE}Hz stream to stdout...\n")

    try:
        while True:
            loop_start = time.time()

            # 1. SNAPSHOT
            data = state.get_snapshot()

            # 2. MATH
            # Gauge
            gauge = (data['d1'] * math.sin(ANGLE1)) + (data['d2'] * math.sin(ANGLE2))
            
            # Cross Level
            tilt = math.atan2(data['ay'], data['az']) if data['az'] != 0 else 0
            cross = math.sin(tilt) * gauge

            # Twist
            twist, current_time = calculate_twist(cross, prev_cross, data['velocity'], prev_time)
            
            # Update history
            prev_cross = cross
            prev_time = current_time

            # 3. PREPARE OUTPUT
            output = {
                "ts": round(current_time, 3),
                "vel": data['velocity'],
                "gauge": round(gauge, 2),
                "cross": round(cross, 2),
                "twist": round(twist, 2),
                "d1": data['d1'],
                "d2": data['d2']
            }

            # 4. SEND TO PIPE (JSON)
            print(json.dumps(output), flush=True)

            # 5. LOG TO CSV
            logger.log([
                output['ts'], output['vel'], 
                output['gauge'], output['cross'], output['twist'],
                data['d1'], data['d2'], 
                round(data['ay'],3), round(data['az'],3)
            ])

            # 6. SLEEP (Maintain Refresh Rate)
            elapsed = time.time() - loop_start
            delay = (1.0 / WEB_REFRESH_RATE) - elapsed
            if delay > 0:
                time.sleep(delay)

    except KeyboardInterrupt:
        sys.stderr.write("\nStopping...\n")
        logger.close()

if __name__ == "__main__":
    main()
