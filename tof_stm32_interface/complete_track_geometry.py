import time

# Path to the pipe
PIPE_PATH = "/tmp/xsens_pipe"

def get_imu_data():
    print("Connecting to IMU driver...")
    with open(PIPE_PATH, "r") as pipe:
        while True:
            # Read line from the pipe
            line = pipe.readline()
            if not line: break
            
            # Process the line (example parsing)
            # Assuming your C++ outputs: "CNT: 123 ACC: 0.1 0.2 0.3..."
            print(f"Received from other folder: {line.strip()}")
                          
            # Add your logic here to sync with camera
if __name__ == "__main__":
    get_imu_data()

