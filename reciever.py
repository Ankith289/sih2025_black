#!/usr/bin/env python3
import sys
import json
import asyncio
import websockets
import time

WS_URL = "ws://192.168.0.10:4000/ws/track-data"

# ============================================================
# ASYNC SENDER TO THE WEB BACKEND
# ============================================================

async def send_to_websocket(queue):
    """Connects to WS and sends packets from the queue."""
    while True:
        try:
            print(f"[WS] Connecting to {WS_URL} ...")
            async with websockets.connect(WS_URL, max_size=10_000_000) as ws:
                print("[WS] Connected!")

                # continuously send packets as they arrive
                while True:
                    packet = await queue.get()  # wait for new data
                    await ws.send(packet)
                    print("[WS] Sent:", packet[:100], "...")
        
        except Exception as e:
            print(f"[WS] Error: {e}. Reconnecting in 2 sec...")
            await asyncio.sleep(2)  # retry

# ============================================================
# STDIN READER (READS FROM CODE-B)
# ============================================================

async def read_stdin(queue):
    """Reads fused JSON packets from Code-B via stdin."""
    loop = asyncio.get_running_loop()

    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            await asyncio.sleep(0.001)
            continue

        line = line.strip()
        if not line:
            continue

        # Validate JSON before sending
        try:
            json.loads(line)
            await queue.put(line)
        except json.JSONDecodeError:
            print("[receiver] Invalid JSON from Code-B:", line)

# ============================================================
# MAIN
# ============================================================

async def main():
    queue = asyncio.Queue()

    await asyncio.gather(
        send_to_websocket(queue),
        read_stdin(queue)
    )

if __name__ == "__main__":
    asyncio.run(main())
