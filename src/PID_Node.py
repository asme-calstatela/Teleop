# !/usr/bin/env python
# PID_Node.py: Starts the PID node for the mobile differential drive
#
# Created By: Jeovanny Reyes
# Created On: February 20, 2018
#
# Cal State LA Robotics Research Laboratory

from pid_vel import PidVelocity

def main():
    rospy.init_node("pid_velocity")
    pid = PidVelocity() # PidVelocity is a class

if __name__ == '__main__':
    """ main """
    try:
        pid.spin()
        #pidVelocity = PidVelocity()
        #pidVelocity.spin()
    except rospy.ROSInterruptException:
pass
