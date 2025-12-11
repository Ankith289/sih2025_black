import zmq
import threading
import time
import math
import json
import sys
import csv
import os
import copy
import collections
import statistics
import re

# ======================================================
# CONFIG
# ======================================================
IMU_PIPE_PATH = "/tmp/xsens_pipe"
CSV_FILE_PATH = "track_data_log.csv"
WEB_REFRESH_RATE = 30
MASTER_ADDR = "tcp://127.0.0.1:6000"    # From uart_time_master

ANGLE1 = math.radians(30)
ANGLE2 = math.radians(60)

# ======================================================
# GLOBAL TIMESTAMP FILE
# ======================================================
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


# ======================================================
# CALCULATION ENGINE
# ======================================================
def calculate_track_parameters(d1, d2, velocity, ay, az, prev_cross, prev_chain, chainage):
    if d2 == 65535:
        d2 = 0

    # Gauge
    gauge = d1 * math.sin(ANGLE1) + d2 * math.sin(ANGLE2)

    # Cross level using IMU tilt
    down = math.sqrt(ay*ay + az*az)
    if 9 < down < 10.1 and abs(az) < 9.0:
        angle = math.asin(az / down)
        cross_level = math.sin(angle) * gauge
    else:
        cross_level = 0.0

    # Twist using Δ chainage
    twist = 0.0
    if prev_chain is not None:
        delta = chainage - prev_chain
        if abs(delta) > 0.001:
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


# ======================================================
# SHARED STATE
# ======================================================
class SystemState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "d1": 0,
            "d2": 0,
            "velocity": 0.0,
            "trigger": 0,
            "chainage": 0.0,
            "ay": 0.0,
            "az": 9.8
        }

    def update_imu(self, ay, az):
        with self.lock:
            self.data["ay"] = ay
            self.data["az"] = az

    def update_zmq(self, pkt):
        with self.lock:
            self.data["d1"] = pkt.get("d1", 0)
            self.data["d2"] = pkt.get("d2", 0)
            self.data["velocity"] = pkt.get("velocity", 0.0)
            self.data["trigger"] = pkt.get("trigger", 0)
            self.data["chainage"] = pkt.get("distance", 0.0)

    def get_snapshot(self):
        with self.lock:
            return copy.deepcopy(self.data)

state = SystemState()


# ======================================================
# IMU WORKER
# ======================================================
def imu_worker():
    while not os.path.exists(IMU_PIPE_PATH): time.sleep(1)
    sys.stderr.write("IMU connected.\n")

    try:
        with open(IMU_PIPE_PATH, 'r') as f:
            while True:
                line = f.readline()
                if not line:
                    continue
                parts = line.split()
                if "ACC:" in parts:
                    try:
                        idx = parts.index("ACC:")
                        ay = float(parts[idx+2])
                        az = float(parts[idx+3])
                        state.update_imu(ay, az)
                    except:
                        pass
    except:
        pass


# ======================================================
# ZMQ SUBSCRIBER (REPLACES UART COMPLETELY)
# ======================================================
def zmq_worker():
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(MASTER_ADDR)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")

    sys.stderr.write(f"[Track Geometry] ZMQ Connected → {MASTER_ADDR}\n")

    while True:
        try:
            pkt = sub.recv_json()
            state.update_zmq(pkt)
        except Exception as e:
            sys.stderr.write(f"[ZMQ] Error: {e}\n")
            time.sleep(0.1)


# ======================================================
# SYNC PACKET TO CODE B
# ======================================================
def build_sync_packet(results):
    return {
        "timestamp": get_global_timestamp(),
        "subsystem": "track_geometry",
        "gauge": results["gauge"],
        "cross_level": results["cross"],
        "twist": results["twist"],
        "unevenness": results["unevenness"],
        "ride_index": results["ride_index"],
        "chainage": results["chainage"],
        "velocity": results["vel"],
        "d1": results["d1"],
        "d2": results["d2"]
    }


# ======================================================
# MAIN LOOP
# ======================================================
def main():
    threading.Thread(target=imu_worker, daemon=True).start()
    threading.Thread(target=zmq_worker, daemon=True).start()

    log_file = open(CSV_FILE_PATH, 'a', newline='')
    writer = csv.writer(log_file)

    if os.stat(CSV_FILE_PATH).st_size == 0:
        writer.writerow(["ts", "vel", "gauge", "cross", "twist",
                         "d1", "d2", "ay", "az", "unevenness",
                         "ride_index", "chainage"])

    prev_cross = 0.0
    prev_chain = 0.0

    ride_quality = RideQualityEstimator(window_seconds=1.0, sample_rate=WEB_REFRESH_RATE)

    sys.stderr.write("Track Geometry running...\n")

    while True:
        loop_start = time.time()

        inputs = state.get_snapshot()
        ay = inputs["ay"]
        az = inputs["az"]

        ride_quality.update(ay, az)
        unevenness, ride_rms = ride_quality.calculate_metrics()

        results, prev_cross, prev_chain = calculate_track_parameters(
            inputs["d1"], inputs["d2"], inputs["velocity"],
            ay, az, prev_cross, prev_chain,
            inputs["chainage"]
        )

        results["unevenness"] = unevenness
        results["ride_index"] = ride_rms

        if inputs["trigger"] == 1:
            print(json.dumps(build_sync_packet(results)), flush=True)

        writer.writerow([
            results['ts'], results['vel'], results['gauge'],
            results['cross'], results['twist'], results['d1'],
            results['d2'], ay, az, unevenness, ride_rms,
            results['chainage']
        ])
        log_file.flush()

        elapsed = time.time() - loop_start
        time.sleep(max(0, (1.0 / WEB_REFRESH_RATE) - elapsed))


# ======================================================
if __name__ == "__main__":
    main()
