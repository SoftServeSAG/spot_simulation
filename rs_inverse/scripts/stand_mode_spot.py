#!/usr/bin/env python


# The file contains hardcoded scripts for robot control in stand mode: change body orientation in relation to foots. 

import rospy
from rs_msgs.msg import GaitInput
import time


def init_pub():
    spot_name = rospy.get_param('~/spot_name')
    rospy.init_node(spot_name + 'stand_mode_inverse', anonymous=True)
    return rospy.Publisher('/' + spot_name + '/inverse_gait_input', GaitInput, queue_size=10)


def command_spot(pub):
    """ Send command to Spot` controller"""
    global xd, yd, zd, rolld, pitchd, yawd
    msg = GaitInput()
    msg.x = float(xd)
    msg.y = float(yd)
    msg.z = float(zd)
    msg.roll = float(rolld)
    msg.pitch = float(pitchd)
    msg.yaw = float(yawd)
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
    global xd, yd, zd, rolld, pitchd, yawd
    xd = 0.0
    yd = 0.0
    zd = 0.0
    rolld = 0.0
    pitchd = 0.0
    yawd = 0.0

    position_seq = [
        [0.1, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.1, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0,
         0.0, 0.0, 0.0, 0.0],
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
         0.2,
         0.2, 0.2, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0,
         0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -0.4, 0.0, 0.0, 0.0, 0.0, 0.3,
         0.0,
         -0.3, 0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.4, 0.0, -0.4, 0.0, 0.5,
         0.0,
         -0.5, 0.0, 0.5, 0.0]]

    pub = init_pub()

    # Initial command
    command_spot(pub)

    for i in range(0, 28):
        time.sleep(2.1)
        xd = position_seq[0][i]
        yd = position_seq[1][i]
        zd = position_seq[2][i]
        rolld = position_seq[3][i]
        pitchd = position_seq[4][i]
        yawd = position_seq[5][i]
        command_spot(pub)



def myhook():
    # Execute on shutdown
    print("Stand mode finished")


if __name__ == '__main__':
    try:
        main()
        rospy.on_shutdown(myhook)
    except rospy.ROSInterruptException:
        pass
