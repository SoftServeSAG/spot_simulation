#!/usr/bin/env python

"""
Forward kinematics is implemented in this scripts.
It is node that publish commands on joints actuators to move Spot in predefined positions
"""

import rospy
import time
from std_msgs.msg import Float64


class SpotForward:
    def __init__(self, spot_name):
        """Initialize the publishers of the joints command"""
        # ---- Front ----
        # left front hip_x
        self.pub_front_left_hip_x = rospy.Publisher('/' + spot_name + '/joint_front_left_hip_x_controller/command',
                                               Float64, queue_size=10)

        # left front hip_y
        self.pub_front_left_hip_y = rospy.Publisher('/' + spot_name + '/joint_front_left_hip_y_controller/command',
                                               Float64, queue_size=10)

        # left front knee
        self.pub_front_left_knee = rospy.Publisher('/' + spot_name + '/joint_front_left_knee_controller/command',
                                              Float64, queue_size=10)

        # right front hip_x
        self.pub_front_right_hip_x = rospy.Publisher('/' + spot_name + '/joint_front_right_hip_x_controller/command',
                                                Float64, queue_size=10)

        # right front hip_y
        self.pub_front_right_hip_y = rospy.Publisher('/' + spot_name + '/joint_front_right_hip_y_controller/command',
                                                Float64, queue_size=10)

        # right front knee
        self.pub_front_right_knee = rospy.Publisher('/' + spot_name + '/joint_front_right_knee_controller/command',
                                               Float64, queue_size=10)

        # ---- Rear ----
        # left rear hip_x
        self.pub_rear_left_hip_x = rospy.Publisher('/' + spot_name + '/joint_rear_left_hip_x_controller/command',
                                              Float64, queue_size=10)

        # left rear hip_y
        self.pub_rear_left_hip_y = rospy.Publisher('/' + spot_name + '/joint_rear_left_hip_y_controller/command',
                                              Float64, queue_size=10)

        # left rear knee
        self.pub_rear_left_knee = rospy.Publisher('/' + spot_name + '/joint_rear_left_knee_controller/command',
                                             Float64, queue_size=10)

        # right rear hip_x
        self.pub_rear_right_hip_x = rospy.Publisher('/' + spot_name + '/joint_rear_right_hip_x_controller/command',
                                               Float64, queue_size=10)

        # right rear hip_y
        self.pub_rear_right_hip_y = rospy.Publisher('/' + spot_name + '/joint_rear_right_hip_y_controller/command',
                                               Float64, queue_size=10)

        # right rear knee
        self.pub_rear_right_knee = rospy.Publisher('/' + spot_name + '/joint_rear_right_knee_controller/command',
                                              Float64, queue_size=10)

    def talker(self, motors_target_pos):
        """ Send command to actuators of joints

        param motors_target_pos: A np.array(4,3) with the values of angels of joints
        """
        # Hips in x-direction
        self.pub_front_left_hip_x.publish(motors_target_pos[0][0])
        self.pub_front_right_hip_x.publish(motors_target_pos[1][0])
        self.pub_rear_left_hip_x.publish(motors_target_pos[2][0])
        self.pub_rear_right_hip_x.publish(motors_target_pos[3][0])
        # Hips in y-direction
        self.pub_front_left_hip_y.publish(motors_target_pos[0][1])
        self.pub_front_right_hip_y.publish(motors_target_pos[1][1])
        self.pub_rear_left_hip_y.publish(motors_target_pos[2][1])
        self.pub_rear_right_hip_y.publish(motors_target_pos[3][1])
        # Knee
        self.pub_front_left_knee.publish(motors_target_pos[0][2])
        self.pub_front_right_knee.publish(motors_target_pos[1][2])
        self.pub_rear_left_knee.publish(motors_target_pos[2][2])
        self.pub_rear_right_knee.publish(motors_target_pos[3][2])


def myhook():
    rospy.loginfo_once("Finished Forward Kinematics")


if __name__ == '__main__':
    try:
        rospy.loginfo_once("Starting Forward Kinematics")
        spot_name = rospy.get_param('~/spot_name')
        rospy.init_node(spot_name + 'talker')
        rate = rospy.Rate(100)  # 100hz
        spot = SpotForward(spot_name)

        # Load standard positions - commands on joints actuators to move Spot in predefined positions
        sit_down = rospy.get_param("~/sit_down")
        stand_up = rospy.get_param("~/stand_up")
        give_paw = rospy.get_param("~/give_paw")
        leg_spread = rospy.get_param("~/leg_spread")
        fl_leg = rospy.get_param("~/fl_leg")
        fr_leg = rospy.get_param("~/fr_leg")
        bow_forward = rospy.get_param("~/bow_forward")
        leg_spread_left = rospy.get_param("~/leg_spread_left")

        pose_dict = {
            "sit_down": sit_down,
            "stand_up": stand_up,
            "give_paw": give_paw,
            "leg_spread": leg_spread,
            "fl_leg": fl_leg,
            "fr_leg": fr_leg,
            "bow_forward": bow_forward,
            "leg_spread_left": leg_spread_left
        }

        standard_position_seq = ["sit_down", "stand_up", "sit_down", "stand_up", "leg_spread", "stand_up", "fl_leg",
                                 "stand_up", "fr_leg", "stand_up", "bow_forward", "stand_up", "leg_spread_left",
                                 "sit_down", "give_paw"]

        rospy.loginfo_once("Started Forward Kinematics")

        for position in standard_position_seq:
            time.sleep(2.1)
            spot.talker(pose_dict[position])

        # sit down
        time.sleep(7.1)
        spot.talker(pose_dict["sit_down"])
        rate.sleep()

        rospy.on_shutdown(myhook)
    except rospy.ROSInterruptException:
        pass
