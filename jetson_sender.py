import websocket
import json
import threading
import time
import uvicorn
from fastapi import FastAPI, Request
from datetime import datetime

# ------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------
DASHBOARD_WS_URL = "ws://localhost:4000/ws/track-data"
ws = None
ws_lock = threading.Lock()

# ------------------------------------------------------------
# MAINTAIN WEBSOCKET CONNECTION (Auto-reconnect)
# ------------------------------------------------------------
def connect_websocket():
    global ws
    while True:
        try:
            print(f"[WS] Connecting to dashboard at {DASHBOARD_WS_URL}")
            ws = websocket.create_connection(DASHBOARD_WS_URL)
            print("[WS] Connected to dashboard!")
            return
        except Exception as e:
            print(f"[WS] Error connecting: {e}. Retrying in 2s...")
            time.sleep(2)

def ws_sender(data):
    """Thread-safe WebSocket sender with auto-reconnect"""
    global ws

    with ws_lock:
        try:
            if ws is None:
                connect_websocket()

            ws.send(json.dumps(data))
        
        except Exception:
            print("[WS] Lost connection. Reconnecting...")
            connect_websocket()
            ws.send(json.dumps(data))

# Start persistent WebSocket connection thread
threading.Thread(target=connect_websocket, daemon=True).start()

# ------------------------------------------------------------
# FASTAPI RECEIVER FROM SIH RUNTIME
# ------------------------------------------------------------
app = FastAPI()

@app.post("/push")
async def receive_from_code_b(request: Request):
    """
    Code B (synchronizer) POSTS track geometry / defects / acceleration here.
    """
    data = await request.json()
    print("[API] Received:", data)

    # Forward to dashboard WebSocket
    ws_sender(data)

    return {"status": "ok"}

# ------------------------------------------------------------
# MAIN ENTRY
# ------------------------------------------------------------
if __name__ == "__main__":
    print("[SYSTEM] Jetson sender starting on port 5001...")
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=5001,
        log_level="info"
    )
