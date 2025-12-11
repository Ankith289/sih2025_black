import serial
import time
import zmq
import re
from datetime import datetime

UART_PORT = "/dev/ttyTHS1"
BAUD = 115200
TS_FILE = "/tmp/global_timestamp.txt"

# ZeroMQ PUB channel
PUB_ADDR = "tcp://127.0.0.1:6000"

# Regex for Blue Pill data
uart_regex = re.compile(
    r"D1:(\d+)\s+D2:(\d+)\s+Velocity:([0-9\.\-]+)\s+Trigger:([0-9])\s+Distance:([0-9\.\-]+)"
)
def write_timestamp(ts):
    """Writes global timestamp for ALL subsystems."""
    try:
        with open(TS_FILE, "w") as f:
            f.write(ts)
    except:
        pass

def parse_uart_line(line: str):
    match = uart_regex.match(line)
    if not match:
        return None

    d1      = int(match.group(1))
    d2      = int(match.group(2))
    vel     = float(match.group(3))
    trigger = int(match.group(4))
    dist    = float(match.group(5))

    return {
        "d1": d1,
        "d2": d2,
        "velocity": vel,
        "trigger": trigger,
        "distance": dist
    }


def main():
    print("[MASTER] Starting UART + Time Publisher...")

    # ---- Setup ZeroMQ PUB ----
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(PUB_ADDR)
    print(f"[MASTER] Publishing on {PUB_ADDR}")

    # ---- Open UART safely ----
    while True:
        try:
            ser = serial.Serial(UART_PORT, BAUD, timeout=1)
            ser.reset_input_buffer()
            print(f"[MASTER] UART connected at {UART_PORT}")
            break
        except Exception as e:
            print(f"[MASTER] UART error: {e}   Retrying...")
            time.sleep(2)

    # ---- Main loop ----
    while True:
        try:
            line = ser.readline().decode("utf-8", "ignore").strip()
            if not line:
                continue

            parsed = parse_uart(line)
            if not parsed:
                continue

            # 1) Write global timestamp file
            write_timestamp(parsed["timestamp"])

            # 2) Publish packet to all subscribers
            pub.send_json(parsed)

        except Exception as e:
            print("[MASTER] Error during read:", e)
            time.sleep(0.2)

if __name__ == "__main__":
    main()
