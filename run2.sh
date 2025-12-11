
# ================================================
# Open TERMINAL 1 → run complete_track_geometry loop
# ================================================
gnome-terminal -- bash -c "
source env/bin/activate;
./xsens_MTI710/mti710_force_log > /tmp/xsens_pipe | python3 tof_stm32_interface/complete_track_geometry.py;
exec bash
"

# ================================================
# Open TERMINAL 2 → run ml.py loop
# ================================================
gnome-terminal -- bash -c "
source envs/bin/activate;
python3 condition/ml.py;
exec bash
"
# ================================================
# Open TERMINAL 3 → run complete_track_geometry loop
# ================================================
gnome-terminal -- bash -c "
source env/bin/activate;
./xsens_MTI710/mti710_force_log > /tmp/xsens_pipe | python3 Acceleration/track_accel.py;
exec bash
"

