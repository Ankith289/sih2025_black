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

def safety_calculation(imu, tofs,previous_cross_level):
    """
    Your custom logic combining 4 distances + Accelerometer
    """
    # Unpack for clarity
    d1, d2, d3, d4 = tofs 
    acc_x = imu['ax']
    acc_y = imu['ay']
    acc_z = imu['az']
    gyro_z = imu['gz']

    decision = "CLEAR"
    
    # 1. Gauge
    gauge_width = (d1*math.sin(angle1))+(d2*math.sin(angle2))
    
    # 2.cross_level
    down= math.sqrt(pow(abs(acc_x),2)+pow(abs(acc_y),2))
    if 9 < down and down < 10 and abs(acc_y)<9:
        cross_level_angle = math.degrees(math.asin(acc_y/down))
        cross_level= math.sin(math.radians(cross_level_angle))*gauge_width
    else:
        cross_level=0
    
    # 3. twist_level
    twist = cross_level - previous_cross_level

    previous_cross_level =cross_level

    

    # 4. curve_level
    

    decision = {"gauge_width": gauge_width,
                "cross_level": cross_level,
                "curve_level": 0
                }

    return decision
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

