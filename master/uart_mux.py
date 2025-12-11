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
regex = re.compile(r"(D1|D2|Velocity|Trigger|Distance)\s*:\s*([\d\.]+)")

def write_timestamp(ts):
    """Writes global timestamp for ALL subsystems."""
    try:
        with open(TS_FILE, "w") as f:
            f.write(ts)
    except:
        pass

def parse_uart(line):
    """Converts UART line into dict."""
    matches = regex.findall(line)
    if not matches:
        return None

    data = {}
    for k, v in matches:
        data[k] = float(v)

    return {
        "timestamp": datetime.now().isoformat(),
        "d1": int(data.get("D1", 0)),
        "d2": int(data.get("D2", 0)),
        "distance": float(data.get("Distance", 0.0)),
        "velocity": float(data.get("Velocity", 0.0)),
        "trigger": int(data.get("Trigger", 0))
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
