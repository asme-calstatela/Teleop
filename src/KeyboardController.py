#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class KeyboardController(object):
    def __init__(self, key_mapping, vel_scales, cmd_topic, keys_topic):
        """
        @param key_mapping: dictionary mapping keystrokes and directions
        @param vel_scales:  list defining velocity scales (TODO)
        @param cmd_topic:   string defining topic to publish Twist messages
        @param keys_topic:  string defining topic to subscribe to keys messages
        @param twist:       Twist message to be published
        """
        self.key_mapping = key_mapping
        self.vel_scales  = vel_scales
        self.last_twist  = Twist() # initializes all values to 0

        self.cmd_pub  = rospy.Publisher(cmd_topic, Twist, queue_size=100)
        self.keys_sub = rospy.Subscriber(keys_topic, String, self.keys_cb, self.twist_pub)


    def set_vels(self, command):
        self.twist.angular.z = self.key_mapping[command][0]
        self.twist.linear.x  = self.key_mapping[command][1]
	self.twist.linear.z  = self.key_mapping[command][2]



    def keys_cb(self, msg, twist_pub):
        if (len(msg.data) ==0):
            return
        vels = self.key_mapping[msg.data[0]]
        self.last_twist.angular.z = vels[0]
        self.last_twist.linear.x  = vels[1]
	self.last_twist.linear.z  = vels[2]
        self.twist_pub(self.last_twist)

    def twist_pub(self, twist):
        self.cmd_pub.publish(twist)
