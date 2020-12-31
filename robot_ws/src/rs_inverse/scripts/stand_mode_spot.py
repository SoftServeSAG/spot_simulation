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
    global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
        PenetrationDepth, SwingPeriod, YawControl, YawControlOn
    msg = GaitInput()
    msg.x = float(xd)
    msg.y = float(yd)
    msg.z = float(zd)
    msg.roll = float(rolld)
    msg.pitch = float(pitchd)
    msg.yaw = float(yawd)
    msg.StepLength = float(StepLength)
    msg.LateralFraction = float(LateralFraction)
    msg.YawRate = float(YawRate)
    msg.StepVelocity = float(StepVelocity)
    msg.ClearanceHeight = float(ClearanceHeight)
    msg.PenetrationDepth = float(PenetrationDepth)
    msg.SwingPeriod = float(SwingPeriod)
    msg.YawControl = float(YawControl)
    msg.YawControlOn = float(YawControlOn)
    pub.publish(msg)


def main():
    global xd, yd, zd, rolld, pitchd, yawd, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, \
        PenetrationDepth, SwingPeriod, YawControl, YawControlOn
    xd = 0.0
    yd = 0.0
    zd = 0.0
    rolld = 0.0
    pitchd = 0.0
    yawd = 0.0
    StepLength = 0.00
    LateralFraction = 0.0
    YawRate = 0.0
    StepVelocity = 0.00
    ClearanceHeight = 0.0
    PenetrationDepth = 0.0
    SwingPeriod = 0.00
    YawControl = 0.0
    YawControlOn = 0.0

    pub = init_pub()

    # Initial command
    command_spot(pub)
    # move body in z-axis
    time.sleep(2.1)
    zd = 0.4
    command_spot(pub)
    time.sleep(2.1)
    zd = 0.0
    command_spot(pub)
    time.sleep(2.1)
    zd = 0.2
    command_spot(pub)
    # move body in x-axis
    time.sleep(2.1)
    xd = 0.1
    command_spot(pub)
    time.sleep(2.1)
    xd = 0.0
    command_spot(pub)
    time.sleep(2.1)
    xd = -0.1
    command_spot(pub)
    time.sleep(2.1)
    xd = 0.0
    command_spot(pub)
    # move body in y-axis
    time.sleep(2.1)
    yd = 0.1
    command_spot(pub)
    time.sleep(2.1)
    yd = 0.0
    command_spot(pub)
    time.sleep(2.1)
    yd = -0.1
    command_spot(pub)
    time.sleep(2.1)
    yd = 0.0
    command_spot(pub)
    # move body roll
    time.sleep(2.1)
    rolld = 0.4
    command_spot(pub)
    time.sleep(2.1)
    rolld = 0.0
    command_spot(pub)
    time.sleep(2.1)
    rolld = -0.4
    command_spot(pub)
    time.sleep(2.1)
    rolld = 0.0
    command_spot(pub)
    # move body pitch
    time.sleep(2.1)
    pitchd = 0.4
    command_spot(pub)
    time.sleep(2.1)
    pitchd = 0.0
    command_spot(pub)
    time.sleep(0.5)
    pitchd = -0.4
    command_spot(pub)
    time.sleep(2.1)
    pitchd = 0.0
    command_spot(pub)
    # move body yaw
    time.sleep(1.1)
    yawd = 0.4
    command_spot(pub)
    time.sleep(2.1)
    yawd = 0.0
    command_spot(pub)
    time.sleep(0.5)
    yawd = -0.4
    command_spot(pub)
    time.sleep(2.1)
    yawd = 0.0
    command_spot(pub)
    # move body in all directions
    time.sleep(1.1)
    yawd = 0.5
    pitchd = 0.3
    command_spot(pub)
    time.sleep(2.1)
    yawd = 0.0
    pitchd = 0.0
    command_spot(pub)
    time.sleep(0.5)
    yawd = -0.5
    pitchd = -0.3
    command_spot(pub)
    time.sleep(2.1)
    yawd = 0.0
    pitchd = 0.0
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
