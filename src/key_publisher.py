#! /usr/bin/env python
"""
Keyboard Driver

This node takes in keystrokes and publishes them as
std_msgs/String. This is an adaptation of
'key_publisher.py' from 'Programming Robots with ROS'
by Quigley, Gerkey, Smart 2015 pg 113.

Input: Keystrokes from the terminal

Output: std_msgs/String that correspond to the keystrokes

Subscribers: None

Publishers: 'keys' with std_msgs/String data type

Written by: Josh Saunders
Written on: 12/23/2016

Modified by:
Modified on:
"""
# External libraries
import sys, select, tty, termios

# ROS imports
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    key_pub = rospy.Publisher('keys', String, queue_size=1)
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)

    # Save attributes
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print "Publishing keystrokes. Press Ctrl-C to exit..."

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] ==[sys.stdin]:
            # Capture one keystroke at a time instead of an entire line
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()

    # Put the console back into standard mode
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
