#!/usr/bin/env python
#
# Client (diff_drive_client) recieves updates from the server (diff_drive_server_node) created in diff_drive_server.cpp file.
# Client is in sync with server and gets updates/refreshed with parameter values every ten seconds
#
# Created By : Jeovanny Reyes
# Created On: 6/3/18
#
# Cal State LA Robotics Laboratory
#

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {Left_Forward_Scale}, {Right_Forward_Scale}, {Left_Backward_Scale}, {Right_Backward_Scale}".format(**config))

if __name__ == "__main__":
    rospy.init_node("diff_drive_client")
    print("Made it past init node")

    #rospy.wait_for_service("diff_drive_server_node/set_parameters") # Don't need to include this
    client = dynamic_reconfigure.client.Client("diff_drive_server_node", timeout=30, config_callback=callback)
    print("Made it past dynamic_reconfigure client command")

    r = rospy.Rate(0.1) # Runs once every ten seconds
    x = 0
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        client.update_configuration({ "Left_Forward_Scale":(1/(x+1)), "Right_Forward_Scale":(1/(x+1)), "Left_Backward_Scale":(1/(x+1)), "Right_Backward_Scale":(1/(x+1)) })
        r.sleep()
