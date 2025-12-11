import websocket
import json
import time
import random
import threading
import cv2
import base64
from datetime import datetime
from fastapi import FastAPI, Request
import uvicorn
import websocket
import json
import time
import threading
# ------------------------------------------------------------
# CONNECT TO DASHBOARD WEBSOCKET
# ------------------------------------------------------------
WS_URL = "ws://localhost:4000/ws/track-data"
ws = None

ws_thread = threading.Thread(target=connect_websocket)
ws_thread.start()

def connect_websocket():
    global ws
    while True:
        try:
            print(f"Connecting to dashboard WS: {WS_URL}")
            ws = websocket.create_connection(WS_URL)
            print("Connected to dashboard!")
            break
        except Exception as e:
            print(f"WS connect error: {e}, retrying...")
            time.sleep(2)



# ------------------------------------------------------------
# FASTAPI SERVER TO RECEIVE DATA FROM CODE B
# ------------------------------------------------------------
app = FastAPI()

@app.post("/push")
async def receive_from_code_b(request: Request):
    global ws
    data = await request.json()

    print("Received processed data from Code B:", data)

    if ws:
        try:
            ws.send(json.dumps(data))
        except Exception:
            connect_websocket()

    return {"status": "ok"}

# ------------------------------------------------------------
# RUN SERVER TO LISTEN ON PORT 5001
# ------------------------------------------------------------

# --- CONFIGURATION ---
# Use "localhost" if running on same PC, or IP if on separate device
SERVER_URL = "ws://localhost:4000/ws/track-data" 

ws = None


def send_data_loop():
    """
    Simulates sending data from various subsystems (Geometry, Rail Profile, etc.)
    """
    chainage = 1000.0
    
    # Mock GPS Start
    lat = 12.9716 
    lon = 77.5946

    while True:
        if ws and ws.connected:
            try:
                # Increment Chainage & GPS
                chainage += 0.5
                lat += 0.0001
                lon += 0.0001
                timestamp = datetime.now().isoformat()

                # --- 1. TRACK GEOMETRY SIMULATION ---
                # Logic: Every 500m, enter a "Curve" zone for 100m
                is_curve = (chainage % 500) < 100 
                if is_curve:
                    curve_radius = random.uniform(300, 800) # Curve Radius
                else:
                    curve_radius = 0 # Straight Track (Tangent)

                geometry_data = {
                    "type": "track_geometry",
                    "gauge": round(1435 + random.uniform(-5, 5), 2),
                    "alignment": round(random.uniform(-2, 2), 2),
                    "cross_level": round(random.uniform(-3, 3), 2),
                    "twist": round(random.uniform(-2, 2), 2),
                    "unevenness": round(random.uniform(0, 3), 2),
                    "curve_radius": round(curve_radius, 2), # SENDING REAL VALUE NOW
                    "chainage": round(chainage, 1)
                }
                ws.send(json.dumps(geometry_data))

                # --- 2. ACCELERATION SIMULATION (Occasional) ---
                if random.random() < 0.2: # 20% chance per loop
                    accel_data = {
                        "type": "acceleration",
                        "vertical": round(random.uniform(0, 0.5), 3),
                        "lateral": round(random.uniform(0, 0.3), 3),
                        "chainage": round(chainage, 1)
                    }
                    ws.send(json.dumps(accel_data))

                # --- 3. COMPONENT DEFECT SIMULATION (Rare) ---
                if random.random() < 0.05: # 5% chance
                    # Mock Image
                    img_data = "https://placehold.co/100x50/red/white?text=Defect"
                    
                    comp_data = {
                        "type": "track_component",
                        "component_type": random.choice(["fastener", "sleeper", "rail_surface", "ballast"]),
                        "status": "defective",
                        "defect_type": "crack",
                        "severity": "high",
                        "chainage": round(chainage, 1),
                        "latitude": round(lat, 6),
                        "longitude": round(lon, 6),
                        "timestamp": timestamp,
                        "image_url": img_data
                    }
                    ws.send(json.dumps(comp_data))
                    print(f"Sent Defect at {chainage}m")

                time.sleep(0.5) # Send 2 updates per second

            except Exception as e:
                print(f"Error sending data: {e}")
                ws.close()
                connect_websocket()
        else:
            connect_websocket()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=5001)
    # Start the connection
    connect_websocket()
    # Start the data loop
    send_data_loop()