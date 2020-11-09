# ground-drone
EECS 149 (Embedded Systems) Project - live remote ground-based drone using PID controller

## Development
0. Update pip: `pip3 install --upgrade pip setuptools wheel`. 
1. Clone the repo: `git clone https://github.com/ngopaul/ground-drone.git`.
2. Create a virtual environment for python3: `python3 -m venv env`, and activate it: `source tutorial-env/bin/activate`.
3. Install the python dependencies from requirements.txt: `pip3 install -r requirements.txt`.

## Running the interface
1. Run app.py with python3, on a raspberry pi with camera plugged in, and with access to the internet.
2. Access localhost:5000 on the pi to view the interface, or access the ip of the pi through the LAN.
