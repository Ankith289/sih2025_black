import time
import json
import os
import copy
from datetime import datetime
import sys

# ============================
# GLOBAL TIMESTAMP READER
# ============================
TS_FILE = "/tmp/global_timestamp.txt"
_last_ts = "NO_TIMESTAMP"
_last_read = 0

def get_global_timestamp():
    global _last_ts, _last_read
    now = time.time()

    # Read timestamp max 200Hz
    if now - _last_read >= 0.005:
        try:
            with open(TS_FILE, "r") as f:
                _last_ts = f.read().strip()
        except:
            _last_ts = "NO_TIMESTAMP"
        _last_read = now

    return _last_ts


# ============================
# IMU PIPE READER
# ============================
IMU_PIPE_PATH = "/tmp/xsens_pipe"

class IMUState:
    def __init__(self):
        self.ax = 0.0
        self.ay = 0.0
        self.az = 9.8   # default gravity

imu_state = IMUState()

def imu_worker():
    """Reads AX AY AZ from XSens pipe continuously."""
    while not os.path.exists(IMU_PIPE_PATH):
        time.sleep(0.2)

    sys.stderr.write("[ACC] IMU Connected.\n")

    with open(IMU_PIPE_PATH, "r") as f:
        while True:
            line = f.readline()
            if not line:
                continue

            parts = line.split()

            # Example XSens packet:
            # "ACC:  AX 0.12  AY -0.03  AZ 9.75"
            if "ACC:" in parts:
                try:
                    i = parts.index("ACC:")
                    imu_state.ax = float(parts[i+1])
                    imu_state.ay = float(parts[i+2])
                    imu_state.az = float(parts[i+3])
                except:
                    continue


# ============================
# SYNC PACKET BUILDER
# ============================
def build_acc_packet(ax, az):
    return {
        "timestamp": get_global_timestamp(),
        "subsystem": "acceleration",
        "ax": round(ax, 3),
        "az": round(az, 3)
    }


# ============================
# MAIN LOOP
# ============================
def main():
    sys.stderr.write("[ACC] Starting Acceleration Subsystem...\n")

    # Start IMU reader
    import threading
    threading.Thread(target=imu_worker, daemon=True).start()

    REFRESH_HZ = 50  # send high frequency data for graph
    interval = 1.0 / REFRESH_HZ

    while True:
        ax = imu_state.ax
        az = imu_state.az

        packet = build_acc_packet(ax, az)

        # Print JSON for Code B (this is the synchronizer input)
        print(json.dumps(packet), flush=True)

        time.sleep(interval)


if __name__ == "__main__":
    main()
