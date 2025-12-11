import time
import requests
import json
from fastapi import FastAPI, Request
import uvicorn

app = FastAPI()

# Storage for 6 subsystems
data_buf = {
    "track_geometry": None,
    "rail_profile": None,
    "track_component": None,
    "acceleration": None,
    "infringement": None,
    "rear_video": None   # video handled separately (optional)
}

# ------------------------------------------------------------
# RECEIVE SUBSYSTEM DATA
# ------------------------------------------------------------
@app.post("/subsystem/{name}")
async def recv_subsystem(name: str, request: Request):
    if name not in data_buf:
        return {"error": "invalid subsystem"}

    payload = await request.json()
    payload["_server_ts"] = time.time()
    data_buf[name] = payload

    fused = synchronize()
    if fused:
        push_to_code_a(fused)

    return {"status": "ok"}

# ------------------------------------------------------------
# TIMESTAMP SYNCHRONIZATION LOGIC
# ------------------------------------------------------------
def synchronize():
    non_video = ["track_geometry", "rail_profile", "track_component", "acceleration", "infringement"]

    # Check if all subsystems have data
    if any(data_buf[s] is None for s in non_video):
        return None

    ts_vals = [data_buf[s]["timestamp"] for s in non_video]

    if max(ts_vals) - min(ts_vals) > 0.05:
        return None  # timestamps not aligned

    return {
        "type": "fused_packet",
        "timestamp": time.time(),
        "track_geometry": data_buf["track_geometry"],
        "rail_profile": data_buf["rail_profile"],
        "track_component": data_buf["track_component"],
        "acceleration": data_buf["acceleration"],
        "infringement": data_buf["infringement"]
    }

# ------------------------------------------------------------
# SEND PROCESSED DATA TO CODE A (WEBSOCKET HANDLER)
# ------------------------------------------------------------
def push_to_code_a(data):
    try:
        requests.post("http://localhost:5001/push", json=data)
        print("Sent fused data to Code A")
    except Exception as e:
        print("Error sending to Code A:", e)

# ------------------------------------------------------------
# RUN ON PORT 5000
# ------------------------------------------------------------
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=5000)
