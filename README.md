# ground-drone
EECS 149 (Embedded Systems) Project - live remote ground-based drone with human tracking using PID controller

## Goals
1. Remote autonomy in the case of a loss of communication with a remote controller.
   - The drone should be able to do advanced actions autonomously. One of the most difficult of these actions is to follow a human, which we will implement.
   - 'Human following' is defined rigorously as finding a human in the robots field of vision and staying within a fixed minimum and maximum distance to the human.
2. Creation of a virtual presence.
   - Remotely view what the drone is seeing and be able to move the drone with remote control.
   - Sometimes packets will be lost - make the controller respond smoothly when there is no message received
  
## Hardware Requirements:
- Raspberry Pi 3B+ or higher (requirement based on RAM for image processing and Wi-Fi)
- TI RSLK MAX kit (https://training.ti.com/ti-robotics-system-learning-kit-max). A cheaper alternative with fewer features can be built using parts from a vendor such as https://protosupplies.com/.
- 3D printer/miscellaneous material to mount the Raspberry Pi to the robot
- Servo with Analog Feedback (https://www.adafruit.com/product/1404)
- Ultrasonic Rangefinder (https://www.adafruit.com/product/982)
- Raspberry Pi Camera V2
- Accessories to support development for RPi and MSP432 (microprocessor for the TI RSLK MAX kit)

## Software Requirements:
- Code Composer Studio (https://www.ti.com/tool/CCSTUDIO)

## Development

### Programming the MSP432
1. Clone into the github repo: `git clone https://github.com/ngopaul/ground-drone.git`.
2. Follow the online training: https://training.ti.com/node/1139680 (only setup required)
   - setup is the first lab: https://www.ti.com/lit/ml/sekp066/sekp066.pdf (required)
      - the zip file mentioned in the lab must be downloaded. It contains libraries which provide basic hardware functions (UART FIFO, LED interface, etc).
3. Open the project stored in the github repo.
4. Adding any functionality which use basic hardware functions must copy the header and c files from the zip's inc/ folder into the project, and then fix all include errors (by removing "../inc/" from includes)

### Programming the Raspberry Pi
1. Update pip: `pip3 install --upgrade pip setuptools wheel`. 
2. Clone the repo: `git clone https://github.com/ngopaul/ground-drone.git`.
3. Create a virtual environment for python3: `python3 -m venv env`, and activate it: `source env/bin/activate`.
4. Install the python dependencies from requirements.txt: `pip3 install -r requirements.txt`.

#### Notes on files:
- `haar-cascade.py` is run with a command-line argument to a haar cascade file, such as `haarcascade/haarcascade_frontalface_alt2.xml`.
- `uart.py` takes in user input and outputs null-terminated (\0) strings over UART
- Plan `app.py` to be the app which:
   - communicates over Wi-Fi to user
   - decides and stores mode for car to be in (user-controlled vs. human follower)
   - communicates mode to MSP432, as well as any extra data.
      - for user-controlled mode, extra data = last received packet, age of last received packet
      - for human follower mode, extra data = angle of human/unknown, distance of human/unknown

## Running the drone
1. Turn on the drone base, which should provide 5V power to the entire system.
2. (a startup script should automatically) run app.py with python3 on the Raspberry Pi. Ensure Wi-Fi connectivity.
3. Access the pi through the LAN.
