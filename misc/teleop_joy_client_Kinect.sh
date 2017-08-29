#!/bin/bash
#This script is for the client_Kinect

export ROS_MASTER_URI='http://192.168.1.193:11311'
#The address might be different so do hostname -I on your computer
export ROS_IP='192.168.1.194'

roscore

# Activate the rqt GUI for The Teleoperation
echo "Launching Teleoperation GUI perspective"
rqt --perspective-file Teleoperation.perspective

# Launching ROS Packages
roslaunch teleop teleop_joy_client_Kinect.launch
# Include package for ultrasounds, IMU, etc.
