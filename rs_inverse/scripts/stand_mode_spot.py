#!/usr/bin/env python


# The file contains hardcoded scripts for robot control in stand mode: change body orientation in relation to foots. 

import rospy
from rs_msgs.msg import GaitInput
import time


def init_pub():
    spot_name = rospy.get_param('~/spot_name')
    rospy.init_node(spot_name + 'stand_mode_inverse', anonymous=True)
    return rospy.Publisher('/' + spot_name + '/inverse_gait_input', GaitInput, queue_size=10)


def command_spot(pub, pos):
    """ Send command to Spot` controller"""
    msg = GaitInput()
    msg.x = float(pos[0])
    msg.y = float(pos[1])
    msg.z = float(pos[2])
    msg.roll = float(pos[3])
    msg.pitch = float(pos[4])
    msg.yaw = float(pos[5])
    msg.StepLength = float(0)
    msg.LateralFraction = float(0)
    msg.YawRate = float(0)
    msg.StepVelocity = float(0)
    msg.ClearanceHeight = float(0)
    msg.PenetrationDepth = float(0)
    msg.SwingPeriod = float(0)
    msg.YawControl = float(0)
    msg.YawControlOn = float(0)
    pub.publish(msg)


def main():
    position_seq = [[0.1, 0.0, 0.2, 0.0, 0.0, 0.0],  # 0 [x, y, z, pitch, roll, yaw]
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 1
                    [-0.1, 0.0, 0.2, 0.0, 0.0, 0.0],  # 2
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 3
                    [0.0, 0.1, 0.2, 0.0, 0.0, 0.0],  # 4
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 5
                    [0.0, -0.1, 0.2, 0.0, 0.0, 0.0],  # 6
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 7
                    [0.0, 0.0, 0.4, 0.0, 0.0, 0.0],  # 8
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 9
                    [0.0, 0.0, 0.05, 0.0, 0.0, 0.0],  # 10
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 11
                    [0.0, 0.0, 0.2, 0.4, 0.0, 0.0],  # 12
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 13
                    [0.0, 0.0, 0.2, -0.4, 0.0, 0.0],  # 14
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 15
                    [0.0, 0.0, 0.2, 0.0, 0.4, 0.0],  # 16
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 17
                    [0.0, 0.0, 0.2, 0.0, -0.4, 0.0],  # 18
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 19
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.4],  # 20
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 21
                    [0.0, 0.0, 0.2, 0.0, 0.0, -0.4],  # 22
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],  # 23
                    [0.0, 0.0, 0.2, 0.0, 0.3, 0.5],    # 24
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0],    # 25
                    [0.0, 0.0, 0.2, 0.0, -0.3, -0.5],  # 26
                    [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]]  # 27

    pub = init_pub()

    # Initial command
    command_spot(pub, [0.0, 0.0, 0.2, 0.0, 0.0, 0.0])

    for i in range(0, len(position_seq)):
        time.sleep(2.1)
        command_spot(pub, position_seq[i])



def myhook():
    # Execute on shutdown
    print("Stand mode finished")


if __name__ == '__main__':
    try:
        main()
        rospy.on_shutdown(myhook)
    except rospy.ROSInterruptException:
        pass
