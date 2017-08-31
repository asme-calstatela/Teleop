#! /usr/bin/env python
"""
Keystrokes to Twist

This node takes in keystrokes along the 'keys' topic and
publishes velocity commands based on the keystrokes.
This is an adaptation of
'keys_to_twist.py' from 'Programming Robots with ROS'
by Quigley, Gerkey, Smart 2015 pgs 116:124.

Input: std_msgs/String that correspond to the keystrokes

Output: geometry_msgs/Twist that correspond to velocities

Subscribers: 'keys' with std_msgs/String data type

Publishers: 'cmd_vel' with geometry_msgs/Twist data type

Written by: Josh Saunders
Written on: 12/23/2016

Modified by:
Modified on:
"""
# ROS imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from KeyboardController import KeyboardController

if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    key_mapping = { 'w': [0, 1,0],  'x': [0, -1,0],
                    'a': [-1, 0,0], 'd': [1, 0,0],
                    's': [0, 0,0], 'p':[0,0,1], 'k':[0,0,-2],'l':[0,0,2], 'n':[0,0,-3],'m':[0,0,3],}
    vel_scales = [1, 1]
    cmd_topic = 'cmd_vel'
    keys_topic = 'keys'

    kbc = KeyboardController(key_mapping, vel_scales, cmd_topic, keys_topic)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
