#!/bin/bash
set -e

echo "hi"
chmod +x xsens_MTI710/permissions.sh
./xsens_MTI710/permissions.sh
sleep 5

# -------------------------------
# ENV 1: For complete_track_geometry
# -------------------------------
python -m venv env
source env/bin/activate
pip install -r xsens_MTI710/requirements.txt

g++ xsens_MTI710/raw_data_reciver.cpp -o xsens_MTI710/mti710_force_log -lusb-1.0 -O2 -pthread

rm -f /tmp/xsens_pipe
mkfifo /tmp/xsens_pipe

rm -f /tmp/track_geo
mkfifo /tmp/track_geo

deactivate

# -------------------------------
# ENV 2: For ml.py
# -------------------------------
python -m venv envs
source envs/bin/activate
pip install -r condition/requirements.txt
deactivate

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

echo "Both processes launched in separate terminals."
