#!/usr/bin/env python

""" Create the node of Spot` controller. To send command to controller, run the node with GUI gui_spot.py"""

import numpy as np
import copy
import rospy
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Float64
from rs_msgs.msg import GaitInput
from gazebo_msgs.msg import ModelStates, ContactsState, ContactState

from SpotKinematics import SpotModel
from Bezier import BezierGait


class SpotControl:
    """
    Realise control of Spot initialising publishers, subscribing on GUI topic.
    """
    def __init__(self, time_step):
        self.spot_name = rospy.get_param('~/spot_name')
        rospy.init_node(self.spot_name + '_inverse')
        self.rate = rospy.Rate(100)  # 100hz
        self.time_step = time_step

        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=self.time_step)

        # ------------------ Inputs for Bezier Gait control ----------------
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.0
        self.rolld = 0.0
        self.pitchd = 0.0
        self.yawd = 0.0
        self.StepLength = 0.00
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        self.StepVelocity = 0.00
        self.ClearanceHeight = 0.0
        self.PenetrationDepth = 0.0
        self.SwingPeriod = 0.00
        self.YawControl = 0.0
        self.YawControlOn = True
        self.Ts_const = False

        # ------------------ Spot states ----------------
        self.x_inst = 0.
        self.y_inst = 0.
        self.z_inst = 0.
        self.roll_inst = 0.
        self.pitch_inst = 0.
        self.yaw_inst = 0.
        self.search_index = -1

        # ------------------ Outputs of Contact sensors ----------------
        self.front_left_lower_leg_contact = 1
        self.front_right_lower_leg_contact = 1
        self.rear_left_lower_leg_contact = 1
        self.rear_right_lower_leg_contact = 1
        self.chattering_front_left_lower_leg_contact = 0
        self.chattering_front_right_lower_leg_contact = 0
        self.chattering_rear_left_lower_leg_contact = 0
        self.chattering_rear_right_lower_leg_contact = 0
        self.lim_chattering = 4

        # -------------------------------------- Publishers -------------------------------------------------------
        # ----Front ----
        # left front hip_x
        self.pub_front_left_hip_x = rospy.Publisher('/' + self.spot_name + '/joint_front_left_hip_x_controller/command',
                                                    Float64, queue_size=1)
        # left front hip_y
        self.pub_front_left_hip_y = rospy.Publisher('/' + self.spot_name + '/joint_front_left_hip_y_controller/command',
                                                    Float64, queue_size=1)
        # left front knee
        self.pub_front_left_knee = rospy.Publisher('/' + self.spot_name + '/joint_front_left_knee_controller/command',
                                                   Float64, queue_size=1)
        # right front hip_x
        self.pub_front_right_hip_x = rospy.Publisher(
            '/' + self.spot_name + '/joint_front_right_hip_x_controller/command',
            Float64, queue_size=1)
        # right front hip_y
        self.pub_front_right_hip_y = rospy.Publisher(
            '/' + self.spot_name + '/joint_front_right_hip_y_controller/command', Float64, queue_size=1)
        # right front knee
        self.pub_front_right_knee = rospy.Publisher('/' + self.spot_name + '/joint_front_right_knee_controller/command',
                                                    Float64, queue_size=1)

        # ---- Rear ----
        # left rear hip_x
        self.pub_rear_left_hip_x = rospy.Publisher('/' + self.spot_name + '/joint_rear_left_hip_x_controller/command',
                                                   Float64, queue_size=1)
        # left rear hip_y
        self.pub_rear_left_hip_y = rospy.Publisher('/' + self.spot_name + '/joint_rear_left_hip_y_controller/command',
                                                   Float64, queue_size=1)
        # left rear knee
        self.pub_rear_left_knee = rospy.Publisher('/' + self.spot_name + '/joint_rear_left_knee_controller/command',
                                                  Float64, queue_size=1)
        # right rear hip_x
        self.pub_rear_right_hip_x = rospy.Publisher('/' + self.spot_name + '/joint_rear_right_hip_x_controller/command',
                                                    Float64, queue_size=1)
        # right rear hip_y
        self.pub_rear_right_hip_y = rospy.Publisher('/' + self.spot_name + '/joint_rear_right_hip_y_controller/command',
                                                    Float64, queue_size=1)
        # right rear knee
        self.pub_rear_right_knee = rospy.Publisher('/' + self.spot_name + '/joint_rear_right_knee_controller/command',
                                                   Float64, queue_size=1)

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

    # ------------- Subscribers ------------
    def callback_gait(self, data):
        """Read the data from GUI (command on controller)"""
        self.xd = data.x
        self.yd = data.y
        self.zd = data.z
        self.rolld = data.roll
        self.pitchd = data.pitch
        self.yawd = data.yaw
        self.StepLength = data.StepLength
        self.LateralFraction = data.LateralFraction
        self.YawRate = data.YawRate
        self.StepVelocity = data.StepVelocity
        self.ClearanceHeight = data.ClearanceHeight
        self.PenetrationDepth = data.PenetrationDepth
        self.SwingPeriod = data.SwingPeriod
        self.YawControl = data.YawControl
        self.YawControlOn = data.YawControlOn
        self.Ts_const = data.Ts_const

    def callback_model(self, data):
        """ Read the data of Spot` positions an orientations"""
        if self.search_index == -1:
            self.search_index = data.name.index(self.spot_name)
        self.x_inst = data.pose[self.search_index].position.x
        self.y_inst = data.pose[self.search_index].position.y
        self.z_inst = data.pose[self.search_index].position.z
        orientation_q = data.pose[self.search_index].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll_inst, self.pitch_inst, self.yaw_inst) = euler_from_quaternion(orientation_list)

    def callback_front_left_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_front_left_lower_leg_contact += 1
            if self.chattering_front_left_lower_leg_contact > self.lim_chattering:
                self.front_left_lower_leg_contact = 0
        else:
            self.front_left_lower_leg_contact = 1
            self.chattering_front_left_lower_leg_contact = 0

    def callback_front_right_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_front_right_lower_leg_contact += 1
            if self.chattering_front_right_lower_leg_contact > self.lim_chattering:
                self.front_right_lower_leg_contact = 0
        else:
            self.front_right_lower_leg_contact = 1
            self.chattering_front_right_lower_leg_contact = 0

    def callback_rear_left_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_rear_left_lower_leg_contact += 1
            if self.chattering_rear_left_lower_leg_contact > self.lim_chattering:
                self.rear_left_lower_leg_contact = 0
        else:
            self.rear_left_lower_leg_contact = 1
            self.chattering_rear_left_lower_leg_contact = 0

    def callback_rear_right_lower_leg_contact(self, data):
        if len(data.states) == 0:
            self.chattering_rear_right_lower_leg_contact += 1
            if self.chattering_rear_right_lower_leg_contact > self.lim_chattering:
                self.rear_right_lower_leg_contact = 0
        else:
            self.rear_right_lower_leg_contact = 1
            self.chattering_rear_right_lower_leg_contact = 0

    def listener(self):
        """ Initialise subscribers"""
        rospy.Subscriber("/" + self.spot_name + "/inverse_gait_input", GaitInput, self.callback_gait)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_model)
        rospy.Subscriber("/" + self.spot_name + "/front_left_lower_leg_contact", ContactsState,
                         self.callback_front_left_lower_leg_contact)
        rospy.Subscriber("/" + self.spot_name + "/front_right_lower_leg_contact", ContactsState,
                         self.callback_front_right_lower_leg_contact)
        rospy.Subscriber("/" + self.spot_name + "/rear_left_lower_leg_contact", ContactsState,
                         self.callback_rear_left_lower_leg_contact)
        rospy.Subscriber("/" + self.spot_name + "/rear_right_lower_leg_contact", ContactsState,
                         self.callback_rear_right_lower_leg_contact)

    def yaw_control(self):
        """ Yaw body controller"""
        yaw_target = self.YawControl
        thr = np.pi / 2
        if (yaw_target > thr and self.yaw_inst < -thr) or (self.yaw_inst > thr and yaw_target < -thr):
            residual = (yaw_target - self.yaw_inst) * np.sign(yaw_target - self.yaw_inst) - 2 * np.pi
            yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
        else:
            residual = yaw_target - self.yaw_inst
            yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
        return yawrate_d

    def spot_inverse_control(self):
        """ Spot controller: combines yaw controller and Bezier Gait"""
        pos = np.array([self.xd, self.yd, self.zd])
        orn = np.array([self.rolld, self.pitchd, self.yawd])
        # yaw controller
        if self.YawControlOn == 1.0:
            YawRate_desired = self.yaw_control()
        else:
            YawRate_desired = self.YawRate
        # Update Swing Period
        self.bzg.Tswing = self.SwingPeriod
        contacts = [self.front_left_lower_leg_contact, self.front_right_lower_leg_contact,
                    self.rear_left_lower_leg_contact,
                    self.rear_right_lower_leg_contact]
        # Set time stance equal to swing period
        Ts_const = self.Ts_const
        # Get Desired Foot Poses
        T_bf = self.bzg.GenerateTrajectory(self.StepLength, self.LateralFraction, YawRate_desired,
                                           self.StepVelocity, self.T_bf0, self.T_bf,
                                           self.ClearanceHeight, self.PenetrationDepth,
                                           contacts, Ts_const)
        joint_angles = self.spot.IK(orn, pos, T_bf)
        self.talker(joint_angles)


# ------------------ Standard pose for starting step
sit_down = [[0.20, 1.0, -2.49],  # Front left leg
            [-0.20, 1.0, -2.49],  # Front right leg
            [0.20, 1.0, -2.49],  # Rear left leg
            [-0.20, 1.0, -2.49]]  # Rear right leg

stand_up = [[0.20, 0.7, -1.39],  # Front left leg
            [-0.20, 0.7, -1.39],  # Front right leg
            [0.20, 0.7, -1.39],  # Rear left leg
            [-0.20, 0.7, -1.39]]  # Rear right leg


def main():
    """ The main() function. """
    rospy.loginfo_once("STARTING SPOT TEST ENV")
    time_step = 0.01
    max_timesteps = 4e6
    spot_control = SpotControl(time_step)
    # --- Prepare Spot for walking, posing ---
    # sit down
    time.sleep(2.1)
    spot_control.talker(sit_down)
    spot_control.rate.sleep()
    # stand up
    time.sleep(2.1)
    spot_control.talker(stand_up)
    spot_control.rate.sleep()
    time.sleep(1.1)
    # ---
    t = 0
    spot_control.listener()
    rospy.loginfo_once("STARTED SPOT TEST ENV")

    while t < (int(max_timesteps)):
        start_time = time.time()

        spot_control.spot_inverse_control()

        t += 1
        # elapsed time == sample time
        elapsed_time = time.time() - start_time
        if elapsed_time < spot_control.time_step:
            time.sleep(spot_control.time_step - elapsed_time)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
