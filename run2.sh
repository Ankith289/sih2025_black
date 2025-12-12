# ================================================
# Open TERMINAL 1 → run complete_track_geometry loop
# ================================================
gnome-terminal -- bash -c "
python3 master/uart_time_mux.py;
exec bash
"

# ================================================
# Open TERMINAL 2 → run complete_track_geometry loop
# ================================================
gnome-terminal -- bash -c "
source env/bin/activate;
./xsens_MTI710/mti710_force_log > /tmp/xsens_pipe | python3 tof_stm32_interface/complete_track_geometry.py;
exec bash
"

# ================================================
# Open TERMINAL 3 → run ml.py loop
# ================================================
gnome-terminal -- bash -c "
source envs/bin/activate;
python3 condition/ml.py;
exec bash
"

# ================================================
# Open TERMINAL 4 → run rail profile
# ================================================
gnome-terminal -- bash -c "
source envs/bin/activate;
python3 rear_window/ardu.py;
exec bash
"

# ================================================
# Open TERMINAL 5 → run Acceleration
# ================================================
gnome-terminal -- bash -c "
source envl/bin/activate;
python3 rail_profile/stero.py;
exec bash
"

# ================================================
# Open TERMINAL 6 → run lidar
# ================================================
gnome-terminal -- bash -c "
source envl/bin/activate;
python3 lidar/complete_solution.py;
exec bash
"

# ================================================
# Open TERMINAL 7 → run rearwindow
# ================================================
gnome-terminal -- bash -c "
source envs/bin/activate;
python3 rear_window/ardu.py;
exec bash
"

# ================================================
# Open TERMINAL 8 → run reciever.py
# ================================================
gnome-terminal -- bash -c "
source envr/bin/activate;
python3 reciever.py;
exec bash
"
