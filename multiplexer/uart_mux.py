import serial
import time
import json
import zmq
import re
from datetime import datetime

UART_PORT = "/dev/ttyTHS1"
BAUD = 115200

# ZMQ Publisher Address
PUB_ADDR = "tcp://127.0.0.1:6000"

# Regex for parsing Blue Pill UART
regex = re.compile(r"(D1|D2|Velocity|Trigger)\s*:\s*([\d\.]+)")

def parse_uart_line(line):
    """Parses UART line into dict or None"""
    matches = regex.findall(line)
    if not matches:
        return None
    
    data = {}
    for k, v in matches:
        data[k] = float(v)

    # Standardize keys
    return {
        "d1": int(data.get("D1", 0)),
        "d2": int(data.get("D2", 0)),
        "velocity": data.get("Velocity", 0.0),
        "trigger": int(data.get("Trigger", 0)),
        "timestamp": datetime.now().isoformat()
    }

def main():
    print("[UART-MUX] Starting...")

    # 1. Setup ZMQ Publisher
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(PUB_ADDR)
    print(f"[UART-MUX] Publishing on {PUB_ADDR}")

    # 2. Open UART
    while True:
        try:
            ser = serial.Serial(UART_PORT, BAUD, timeout=1)
            ser.reset_input_buffer()
            print(f"[UART-MUX] Connected to {UART_PORT}")
            break
        except Exception as e:
            print(f"[UART-MUX] UART open failed: {e}. Retrying...")
            time.sleep(2)

    # 3. Main read loop
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            parsed = parse_uart_line(line)
            if parsed:
                pub.send_json(parsed)
                #print("[UART-MUX] Sent:", parsed)

        except Exception as e:
            print("[UART-MUX] Error:", e)
            time.sleep(1)

if __name__ == "__main__":
    main()
