import sys
import json
import time
import threading
from flask import Flask, request, jsonify

# ==========================================================
# CONFIGURATION
# ==========================================================
SYNC_TIMEOUT = 0.050   # 50 ms window allowed for sync fusion
PRINT_FUSED = True     # Send only fused packets to Code A (WS forwarder)

# ==========================================================
# SUBSYSTEM BUFFER
# ==========================================================
subsystems = {
    "track_geometry": None,
    "track_component": None,
    "acceleration": None,
    # ADD MORE SUBSYSTEMS HERE IF YOU NEED:
    # "rear_video": None,
    # "profile_wear": None,
    # "infringement": None,
}

lock = threading.Lock()

# Store when each subsystem last updated
timestamps = {name: 0 for name in subsystems}

# ==========================================================
# FUNCTION: CHECK IF ALL REQUIRED DATA ARRIVED
# ==========================================================
def all_ready():
    with lock:
        return all(subsystems[name] is not None for name in subsystems)

# ==========================================================
# FUNCTION: FUSE DATA FROM ALL SUBSYSTEMS
# ==========================================================
def build_fused_packet():
    with lock:
        fused = {
            "timestamp": subsystems["track_geometry"]["timestamp"],  # master timestamp
            "track_geometry": subsystems["track_geometry"],
            "track_component": subsystems["track_component"],
            "acceleration": subsystems["acceleration"],
            # ADD THE OTHER SUBSYSTEMS HERE
        }
    return fused


# ==========================================================
# FUNCTION: CLEAR BUFFERS AFTER FUSION
# ==========================================================
def clear_subsystems():
    with lock:
        for k in subsystems:
            subsystems[k] = None


# ==========================================================
# THREAD: STDIN SUBSYSTEM READER (Python Local Processes)
# ==========================================================
def stdin_reader():
    """Reads JSON packets from Python subsystems."""
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            pkt = json.loads(line)
            name = pkt.get("subsystem", None)
            ts = pkt.get("timestamp", None)

            if name not in subsystems:
                continue

            with lock:
                subsystems[name] = pkt
                timestamps[name] = time.time()

        except Exception as e:
            sys.stderr.write(f"[CodeB] JSON error: {e}\n")


# ==========================================================
# THREAD: SYNC FUSION LOOP
# ==========================================================
def sync_loop():
    while True:
        time.sleep(0.005)  # 200 Hz check

        if not all_ready():
            continue

        # Check if data timestamps are within timeout window
        t_now = time.time()
        with lock:
            max_age = max(t_now - timestamps[s] for s in subsystems)

        if max_age > SYNC_TIMEOUT:
            # One subsystem is lagging / stale — wait more
            continue

        fused = build_fused_packet()

        # Send fused packet to Code A (via print → pipe)
        if PRINT_FUSED:
            print(json.dumps(fused), flush=True)

        clear_subsystems()


# ==========================================================
# OPTIONAL: HTTP SERVER FOR EXTERNAL SUBSYSTEMS (Port 5000)
# ==========================================================
app = Flask(__name__)

@app.post("/subsystem/<name>")
def http_subsystem(name):
    if name not in subsystems:
        return jsonify({"error": "unknown subsystem"}), 404

    pkt = request.json
    if not pkt:
        return jsonify({"error": "empty packet"}), 400

    with lock:
        subsystems[name] = pkt
        timestamps[name] = time.time()

    return jsonify({"status": "ok"}), 200


# ==========================================================
# MAIN ENTRY POINT
# ==========================================================
if __name__ == "__main__":
    # Start stdin reader
    threading.Thread(target=stdin_reader, daemon=True).start()

    # Start sync engine
    threading.Thread(target=sync_loop, daemon=True).start()

    # Start HTTP server for remote subsystems
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
