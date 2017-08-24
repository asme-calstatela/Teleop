# teleop
This is the catkinized ROS package for the Teleoperation robot at California State University, Los Angeles.

## Installation
Easy! Just clone the repo and compile your workspace!

## Dependencies
This was developed using ROS Indigo and is developed and tested on Ubuntu 14.04. Any other versions of ROS or Ubuntu are not guaranteed to work. There are a couple packages that will need to be installed in order to use this package.

### [joy](http://wiki.ros.org/joy "ROS joy package")
To install `joy` run `sudo apt-get install ros-indigo-joy` or clone the [joy repo](https://github.com/ros-drivers/joystick_drivers "ROS joy package GitHub repo").

### [rosserial](http://wiki.ros.org/rosserial "ROS rosserial package")
This uses `rosserial`, specifically [`rosserial_arduino`](http://wiki.ros.org/rosserial_arduino "ROS rosserial_arduino package"). This should not require any installation on your part as it comes with the full installation of ROS.

## Running It (OLD WAY)
It is recommended that the package be run via `launch` files, although it is possible to run each node individually. However, sometimes it *is* useful to run a node individually due to a connection problem with the Arduino, for example.

### teleop_joy.launch
This launch file connects to the Arduino motor driver, starts the joystick publisher, and the Twist message publisher for the joystick. To run, type `roslaunch teleop teleop_joy.launch` in the terminal.

### teleop_key.launch
This launch file connects to the Arduino motor driver, starts the keyboard publisher, and the Twist message publisher for the keyboard. To run, type `roslaunch teleop teleop_key.launch` in the terminal.

### arduino_driver.launch
This launch file connects to the Arduino motor driver. To run, type `roslaunch teleop arduino_driver.launch` in the terminal.

## Running It (NEW WAY)
This requires two different terminals to be open and execution of two shell scripts.
1st Terminal
`ssh teleoperation@teleoperation`
`source name_of_directory/devel/setup.bash`
`cd name_of_directory/src/teleop/misc`
`./teleop_joy_server_Kinect.sh`
  
2nd Terminal
`source name_of_direcory/devel/setup.bash`
`cd name_of_directory/src/teleop/misc`
`./teleop_joy_client_Kinect.sh`

**TIP:** Should you want to automatically be on the correct path every time you open up a new terminal without having to manually source everytime. DO the following: 
`echo "source ~/name_of_working_directory/devel/setup.bash" >> ~/.bashrc`
`source ~/.bashrc`

## Common Errors
There are many issues that can arise when attempting to run the software.

### Arduino Connection Issues
For some reason, the Arduino has intermittent connection issues. If the Arduino is found by ROS and is connected then there aren't any problems. However, sometimes ROS can't connect to the Arduino. The error message will look something like

`Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino`

A way to solve this is by unplugging the USB cable and plugging it back in. When this is done sometimes the port number will change and you will need to change the port number. A way to check what the new port number is you can open the Arduino IDE and check under `Tools` -> `Port`. If all else fails, restart the computer that is connected to the Arduino and that should solve the problem.

**Note:** You **must** kill the Arduino driver when attempting to upload code to the Arduino by pressing `Ctrl+C`. If you don't, you may create a connection problem with ROS and the Arduino and the computer may require a restart.
