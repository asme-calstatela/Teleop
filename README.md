# teleop
This is the catkinized ROS package for the Teleoperation robot at California State University, Los Angeles.

## Installation
Easy! Just clone the repo!

## Dependencies
This was developed using ROS Indigo and is developed and tested on Ubuntu 14.04. Any other versions of ROS or Ubuntu are not guaranteed to work. There are a couple packages that will need to be installed in order to use this package.

### [joy](http://http://wiki.ros.org/joy "ROS joy package")
To install `joy` run `sudo apt-get install ros-indigo-joy` or clone the [joy repo](http://http://wiki.ros.org/joy "ROS joy package GitHub repo").

### [rosserial](http://http://http://wiki.ros.org/rosserial "ROS rosserial package")
This uses `rosserial`, specifically [`rosserial_arduino`](http://http://http://wiki.ros.org/rosserial "ROS rosserial_arduino package"). This should not require any installation on your part as it comes with the full installation of ROS.

## Running It
It is recommended that the package be run via `launch` files, although it is possible to run each node individually. However, sometimes it *is* useful to run a node individually due to a connection problem with the Arduino, for example.

### teleop_joy.launch
This launch file connects to the Arduino motor driver, starts the joystick publisher, and the Twist message publisher for the joystick. To run, type `roslaunch teleop teleop_joy.launch` in the terminal.

### teleop_key.launch
This launch file connects to the Arduino motor driver, starts the keyboard publisher, and the Twist message publisher for the keyboard. To run, type `roslaunch teleop teleop_key.launch` in the terminal.

### arduino_driver.launch
This launch file connects to the Arduino motor driver. To run, type `roslaunch teleop arduino_driver.launch` in the terminal.

## Common Errors
There are many issues that can arise when attempting to run the software.

### Arduino Connection Issues
For some reason, the Arduino has intermittent connection issues. If the Arduino is found by ROS and is connected then there aren't any problems. However, sometimes ROS can't connect to the Arduino. The error message will look something like

`Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino`

A way to solve this is by unplugging the USB cable and plugging it back in. When this is done sometimes the port number will change and you will need to change the port number. A way to check what the new port number is you can open the Arduino IDE and check under `Tools` -> `Port`. If all else fails, restart the computer that is connected to the Arduino and that should solve the problem.

**Note:** You **must** kill the Arduino driver when attempting to upload code to the Arduino by pressing `Ctrl+C`. If you don't, you may create a connection problem with ROS and the Arduino and the computer may require a restart.
