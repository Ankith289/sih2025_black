import sys
import re
import collections
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- CONFIGURATION ---
WINDOW_SIZE = 300       # Show last 300 packets
UPDATE_INTERVAL = 10    # Update every 10ms (Very fast)

# --- THREAD-SAFE DATA STORAGE ---
# Deques automatically remove old items when they get full
timestamps = collections.deque(maxlen=WINDOW_SIZE)
acc_x = collections.deque(maxlen=WINDOW_SIZE)
acc_y = collections.deque(maxlen=WINDOW_SIZE)
acc_z = collections.deque(maxlen=WINDOW_SIZE)
gyro_x = collections.deque(maxlen=WINDOW_SIZE)
gyro_y = collections.deque(maxlen=WINDOW_SIZE)
gyro_z = collections.deque(maxlen=WINDOW_SIZE)

# Regex matches your specific C++ output format
pattern = re.compile(r"CNT:\s+(\d+).*?ACC:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+).*?GYRO:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)")

running = True

# --- BACKGROUND READER THREAD ---
def data_reader():
    global running
    while running:
        try:
            # This blocks until C++ sends a line
            line = sys.stdin.readline()
            if not line: break 
            
            match = pattern.search(line)
            if match:
                # Parse Data
                t = int(match.group(1)) # Packet Counter
                ax = float(match.group(2))
                ay = float(match.group(3))
                az = float(match.group(4))
                gx = float(match.group(5))
                gy = float(match.group(6))
                gz = float(match.group(7))

                # Append to Deques
                timestamps.append(t)
                acc_x.append(ax)
                acc_y.append(ay)
                acc_z.append(az)
                gyro_x.append(gx)
                gyro_y.append(gy)
                gyro_z.append(gz)
        except ValueError:
            continue
        except Exception:
            break

# --- PLOT SETUP ---
# Use a dark background style for better visibility (optional)
plt.style.use('fast') 
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Plot 1: Acceleration
line_ax, = ax1.plot([], [], 'r-', label='X', lw=1)
line_ay, = ax1.plot([], [], 'g-', label='Y', lw=1)
line_az, = ax1.plot([], [], 'b-', label='Z', lw=1)
ax1.set_title(f"MTi-710 Acceleration (Window: {WINDOW_SIZE})")
ax1.set_ylabel("m/s²")
ax1.legend(loc="upper left")
ax1.grid(True, linestyle='--', alpha=0.6)

# Plot 2: Gyroscope
line_gx, = ax2.plot([], [], 'r-', label='X', lw=1)
line_gy, = ax2.plot([], [], 'g-', label='Y', lw=1)
line_gz, = ax2.plot([], [], 'b-', label='Z', lw=1)
ax2.set_title("Gyroscope")
ax2.set_ylabel("rad/s")
ax2.set_xlabel("Packet Counter (Time)")
ax2.legend(loc="upper left")
ax2.grid(True, linestyle='--', alpha=0.6)

# --- ANIMATION FUNCTION ---
def animate(i):
    if not timestamps: return
    
    # Convert deques to lists for plotting
    t_data = list(timestamps)
    
    # Update Acceleration
    line_ax.set_data(t_data, list(acc_x))
    line_ay.set_data(t_data, list(acc_y))
    line_az.set_data(t_data, list(acc_z))
    
    # Update Gyro
    line_gx.set_data(t_data, list(gyro_x))
    line_gy.set_data(t_data, list(gyro_y))
    line_gz.set_data(t_data, list(gyro_z))
    
    # Dynamic X-Axis limits (The "Scrolling" Effect)
    # We set the view to match the exact range of our current timestamps
    xmin, xmax = min(t_data), max(t_data)
    
    # Add a tiny buffer so lines don't touch the edge
    ax1.set_xlim(xmin, xmax) 
    ax2.set_xlim(xmin, xmax)
    
    # Dynamic Y-Axis limits (Auto-scale vertical)
    ax1.relim()
    ax1.autoscale_view(scalex=False, scaley=True)
    ax2.relim()
    ax2.autoscale_view(scalex=False, scaley=True)

    return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

# --- RUN ---
# Start Reader Thread
t = threading.Thread(target=data_reader)
t.daemon = True
t.start()

# Start Plot
# interval=10 means 10ms (100 FPS target)
# blit=False is safer for dynamic axes scaling
ani = animation.FuncAnimation(fig, animate, interval=UPDATE_INTERVAL, blit=False)
plt.show()
running = False
