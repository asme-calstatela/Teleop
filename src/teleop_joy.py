#! /usr/bin/env python
from TeleopJoy import TeleopJoy
import rospy

def main():
    cmd_topic = "cmd_vel"
    joy_topic = "joy"
    teleop_joy = TeleopJoy(cmd_topic, joy_topic)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
