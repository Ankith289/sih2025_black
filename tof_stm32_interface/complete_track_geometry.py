import math
import os
import time
import threading
import struct
from smbus2 import SMBus

# --- CONFIGURATION ---
PIPE_PATH = "/tmp/xsens_pipe"
I2C_BUS_ID = 1           # Jetson AGX Orin usually uses Bus 1 or 7
TOF_ADDR = 0x42          # <--- Replace with your Blue Pill Address

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
    Reads 4 ToF sensors via I2C.
    Expects 8 bytes total.
    """
    def __init__(self, bus_id, address):
        self.bus_id = bus_id
        self.address = address
        self.bus = None
        # Default values if read fails
        self.distances = [0, 0, 0, 0] 

    def connect(self):
        try:
            self.bus = SMBus(self.bus_id)
            print("TOF: I2C Connected.")
        except Exception as e:
            print(f"TOF: Connection Error: {e}")

    def read(self):
        if not self.bus: return self.distances

        try:
            # Read 8 bytes (4 sensors * 2 bytes each)
            # Register 0x00 is standard, change if your Blue Pill needs a specific reg
            block = self.bus.read_i2c_block_data(self.address, 0x00, 8)
            
            # Unpack 4 unsigned short integers (2 bytes each)
            # >HHHH means Big Endian, 4 items, unsigned short
            # If values look crazy (e.g. 256mm becomes 1mm), change '>' to '<'
            self.distances = list(struct.unpack('>HHHH', bytes(block)))
            
        except OSError:
            # Keep previous values on temporary I2C fail
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
    gyro_x = imu['gx']
    gyro_y = imu['gy']
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
                "twist_level": twist,
                "previous_cross_level": previous_cross_level,
                "curve_level": 0
                }

    return decision

# --- MAIN LOOP ---
if __name__ == "__main__":
    # 1. Start IMU Thread (Reads the Pipe)
    imu_thread = IMUListener(PIPE_PATH)
    imu_thread.start()

    # 2. Setup I2C (Reads the Blue Pill)
    tof_array = ToFArrayReader(I2C_BUS_ID, TOF_ADDR)
    tof_array.connect()

    print("Fusion System Started...")
    
    try:
        while True:
            # A. Get Synchronized Data
            imu_data = imu_thread.get_data()
            tof_data = tof_array.read() # Returns [d1, d2, d3, d4]

            # B. Run Logic
            status = safety_calculation(imu_data, tof_data)

            # C. Print
            # Using f-strings to format nicely
            print(f"TOF: {tof_data} | "
                  f"ACC: {imu_data['ax']:.2f} | "
                  f"STATUS: {status}", end='\r')

            time.sleep(0.05) # 20Hz

    except KeyboardInterrupt:
        print("\nStopping...")
        imu_thread.running = False
