#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoy(object):
    def __init__(self, cmd_topic, joy_topic):
        rospy.init_node("teleop_joy")
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=100)
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joy_cb)

        self.twist = Twist()

    def joy_cb(self, data):
        self.twist.angular.z = data.axes[0]
        self.twist.linear.x = data.axes[1]
        self.cmd_pub.publish(self.twist)
