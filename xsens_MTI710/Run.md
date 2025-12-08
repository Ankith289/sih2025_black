# First

In trminal run 

```
chmod +x permissions.sh 
```

run the permissions.sh file 

# Second

Add the python env file with 
```
python -m venv env
pip install -r requirements.txt
```

Activate the enviornment with 

```
source env/bin/activate
```

# Third

Run
```
g++ mti710_new.cpp -o mti710_force_log -lusb-1.0 -O2 -pthread
```

# Fourth

Run
```
mkfifo /tmp/xsens_pipe
```
# Fifth

```
# Run driver and send output to the pipe
sudo ./mti710_logger > /tmp/xsens_pipe
```
