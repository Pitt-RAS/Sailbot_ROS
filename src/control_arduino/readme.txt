Steps for running code on the Teensy:

- Install Arduino
- Install Teensyduino
- Set up rosserial:
    sudo apt install ros-kinetic-rosserial
    sudo apt install ros-kinetic-rosserial-python
    sudo apt install ros-kinetic-rosserial-arduino

- Install necessary Arduino libraries by cloning the following into Arduino/libraries/<name>
- Install SBUS library:
    https://github.com/bolderflight/SBUS.git
- Wind sensor library:
    https://github.com/DashZhang/AS5045.git
- IMU library:
    https://github.com/adafruit/Adafruit_BNO055.git
- Sensor library:
    https://github.com/adafruit/Adafruit_Sensor.git

- For the first time and every time you add a file, you need to make the Arduino headers for the code:
    rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
    If you've done this before, make sure to delete the Arduino/libraries/ros_lib folder first.

- Upload the code via the Arduino IDE
- Start roscore, then rosserial:
    rosrun rosserial_python serial_node.py /dev/ttyACM0
    
