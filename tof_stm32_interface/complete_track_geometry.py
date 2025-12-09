import math
import os
import time
import threading
import struct
import serial

# --- CONFIGURATION ---
PIPE_PATH = "/tmp/xsens_pipe"

# angles in RADIANS (correct for trigonometry)
angle1 = math.radians(30)
angle2 = math.radians(60)

previous_cross_level = 0.0
prev_time = None


# ===============================
# IMU PIPE THREAD
# ===============================

class IMUListener(threading.Thread):
    def __init__(self, pipe_path):
        super().__init__()
        self.pipe_path = pipe_path
        self.daemon = True
        self.running = True
        self.data = {
            'ax': 0.0, 'ay': 0.0, 'az': 0.0,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
            'qw': 0.0
        }

    def run(self):
        while not os.path.exists(self.pipe_path):
            time.sleep(0.5)

        print(f"IMU: Connected to {self.pipe_path}")

        try:
            with open(self.pipe_path, 'r') as fifo:
                while self.running:
                    line = fifo.readline().strip()
                    if not line:
                        time.sleep(0.01)
                        continue
                    self._parse(line)
        except Exception as e:
            print(f"IMU Pipe Error: {e}")

    def _parse(self, line):
        parts = line.split()
        try:
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
                    self.data['qw'] = float(parts[i+1])
        except:
            pass

    def get_data(self):
        return self.data


# ===============================
# UART ToF Reader
# ===============================

class ToFArrayReader:
    def __init__(self, port="/dev/ttyTHS1", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.distances = [0,0,0,0]

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            print("UART: Connected to Blue Pill.")
        except Exception as e:
            print(f"UART Connection Error: {e}")

    def read(self):
        if not self.ser:
            return self.distances

        try:
            data = self.ser.read(8)
            if len(data) == 8:
                self.distances = list(struct.unpack("<HHHH", data))
        except:
            pass

        return self.distances


# ===============================
# SAFETY GEOMETRY CALCULATION
# ===============================

def safety_calculation(imu, tofs, previous_cross_level, prev_time):
    """
    Pure math version — NO thresholds.
    Computes:
    gauge_width
    cross_level (from tilt)
    twist_level = derivative (dt-based)
    """

    # unpack
    d1, d2, d3, d4 = tofs
    ax, ay, az = imu['ax'], imu['ay'], imu['az']

    # 1) GAUGE WIDTH
    gauge_width = (d1 * math.sin(angle1)) + (d2 * math.sin(angle2))

    # 2) CROSS LEVEL (projection of lateral accel)
    #    We use atan2-based tilt instead of asin to avoid domain issues.
    if az != 0:
        tilt_angle = math.atan2(ay, az)      # radians
    else:
        tilt_angle = 0.0

    cross_level = math.sin(tilt_angle) * gauge_width

    # 3) TIME-BASED TWIST LEVEL
    now = time.time()

    if prev_time is None:
        dt = 0.05
    else:
        dt = max(1e-6, now - prev_time)

    twist_level = (cross_level - previous_cross_level) / dt

    # Construct output
    decision = {
        "gauge_width": gauge_width,
        "cross_level": cross_level,
        "twist_level": twist_level,
    }

    return decision, cross_level, now



# ===============================
# MAIN LOOP
# ===============================

if __name__ == "__main__":

    imu_thread = IMUListener(PIPE_PATH)
    imu_thread.start()

    tof_reader = ToFArrayReader("/dev/ttyTHS1", 115200)
    tof_reader.connect()

    print("Fusion System Started...")

    previous_cross_level = 0.0
    prev_time = None

    try:
        while True:
            imu_data = imu_thread.get_data()
            tof_data = tof_reader.read()

            decision, previous_cross_level, prev_time =
                safety_calculation(imu_data, tof_data, previous_cross_level, prev_time)

            print(f"TOF: {tof_data} | "
                  f"ACC: {imu_data['ax']:.2f} | "
                  f"STATUS: {decision}", end="\r")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
        imu_thread.running = False
