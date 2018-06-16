#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    #rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))
    rospy.loginfo("Config set to {double_param}, {double_param}, {double_param}, {double_param}".format(**config))

if __name__ == "__main__":
    rospy.init_node("diff_drive_client")

    #rospy.wait_for_service("dynamic_tutorials")
    rospy.wait_for_service("diff_drive_server")
    # client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)
    client = dynamic_reconfigure.client.Client("diff_drive", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1) # Runs once every ten seconds
    x = 0
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        # client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
        client.update_configuration({ "double_param":(1/(x+1)), "double_param":(1/(x+1)), "double_param":(1/(x+1)), "double_param":(1/(x+1)) })
        r.sleep()
