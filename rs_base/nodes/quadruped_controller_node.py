#!/usr/bin/env python

import rospy

from rs_base.quadruped_controller import QuadrupedController


if __name__ == "__main__":
    rospy.init_node("quadruped_controller_node", log_level=rospy.INFO)
    controller = QuadrupedController()
    while not rospy.is_shutdown():
        rospy.spin()
