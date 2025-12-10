echo hi
chmod +x xsens_MTI710/permissions.sh 
./permissions
sleep 5
python -m venv env
pip install -r xsens_MTI710/requirements.txt
source env/bin/activate
g++ xsens_MTI710/raw_data_reciver.cpp -o xsens_MTI710/mti710_force_log -lusb-1.0 -O2 -pthread
rm -f /tmp/xsens_pipe
mkfifo /tmp/xsens_pipe
rm -f /tmp/track_geo
mkfifo /tmp/track_geo
./xsens_MTI710/mti710_force_log > /tmp/xsens_pipe |python3 tof_stm32_interface/complete_track_geometry.py > /tmp/track_geo
python -m venv envs
pip install -r condition/requirements.txt
/home/nvidia/env_s/bin/python3 condition/ml.py
