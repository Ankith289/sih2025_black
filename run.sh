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
sudo chown -R $USER:$USER /home/nvidia/sih2025_black/env
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
sudo chown -R $USER:$USER /home/nvidia/sih2025_black/envs
source envs/bin/activate
pip install -r condition/requirements.txt
deactivate

# ENV 3: works on track_geo_env

# -------------------------------
# ENV 2: For ml.py
# -------------------------------

python -m venv envl
sudo chown -R $USER:$USER /home/nvidia/sih2025_black/envs
source envl/bin/activate
pip install -r lidar/req.txt
deactivate

