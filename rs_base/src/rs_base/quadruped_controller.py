#!/usr/bin/env python
import rospy
import tf_conversions
import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from rs_msgs.msg import GaitInput


class QuadrupedController:
    def __init__(self):
        # Const Gait parameters
        self.Tswing = 0.17
        self.ClearanceHeight = 0.15
        self.PenetrationDepth = 0.00001
        # Init gait parameters
        self.StepLength = 0.0
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        # Init body pose command
        self.body_position_x = 0.0
        self.body_position_y = 0.0
        self.body_position_z = 0.0
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.0

        self.time_velocity = 0.0

        # Limits
        self.max_linear_velocity_x = 1.9   # m/sec
        self.max_linear_velocity_y = 0.5  # m/sec

        # Robot name
        robot_name = rospy.get_param("~robot_name", "spot1")

        # Initialize subscriber on velocity command
        twist_topic_name = rospy.get_param("~twist_topic_name", "/cmd_vel")
        self.sub = rospy.Subscriber(twist_topic_name, Twist, self.twist_callback)
        # Initialize subscriber on pose command
        pose_topic_name = rospy.get_param("~pose_topic_name", "/body_pose")
        self.sub = rospy.Subscriber(pose_topic_name, Pose, self.pose_callback)
        # Publisher of gait input
        gait_topic_name = rospy.get_param("~gait_topic_name", "/body_pose")
        self.pub_gait = rospy.Publisher(gait_topic_name, GaitInput, queue_size=10)

        # Time during which no velocity command is received. To stop a robot movement
        self.time_stop = rospy.get_param("~time_stop", 1)
        rospy.Timer(rospy.Duration(0.1), self.check_velocity_update_time)

    def twist_callback(self, vel):
        self.velocity_controller(vel)
        self.time_velocity = rospy.get_time()
        self.send_command()

    def check_velocity_update_time(self, event):
        """ Avoid robot runaway"""
        if rospy.get_time() - self.time_velocity > self.time_stop and self.time_velocity!=0.0:
            self.StepLength = 0.0
            self.send_command()

    def pose_callback(self, pose):
        self.body_position_x = -pose.position.x
        self.body_position_y = pose.position.y
        self.body_position_z = -pose.position.z
        self.body_roll, self.body_pitch, self.body_yaw = tf_conversions.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        self.send_command()

    def velocity_controller(self, vel):
        """ Velocity controller"""
        # Limit linear velocity in x direction
        if abs(vel.linear.y) > self.max_linear_velocity_y:
            velocity_y = self.max_linear_velocity_y * np.sign(vel.linear.y)
        else:
            velocity_y = vel.linear.y
        # Limit linear velocity in y direction
        if abs(vel.linear.x) > self.max_linear_velocity_x:
            velocity_x = self.max_linear_velocity_x * np.sign(vel.linear.x)
        else:
            velocity_x = vel.linear.x

        if velocity_y == 0:
            if velocity_x != 0:
                self.StepLength = self.Tswing / 2 * velocity_x
                self.LateralFraction = 0.0
            else:
                self.StepLength = 0.0
                self.LateralFraction = 0.0
        elif velocity_y > 0:
            self.StepLength = self.Tswing / 2 * velocity_y
            self.LateralFraction = 1.0
        elif velocity_y < 0:
            self.StepLength = self.Tswing / 2 * abs(velocity_y)
            self.LateralFraction = -1.0

        if vel.angular.z != 0.0 and velocity_x == 0.0:
            self.StepLength = 0.1
            # Note! Rotate robot without forward movement
            self.YawRate = 5 * vel.angular.z
            if self.YawRate > 10:
                self.YawRate = 10
        else:
            self.YawRate = vel.angular.z

    def send_command(self):
        """ Publish command to inverse controller"""
        msg = GaitInput()
        msg.x, msg.y, msg.z = self.body_position_x, self.body_position_y, self.body_position_z
        msg.roll, msg.pitch, msg.yaw = self.body_roll, self.body_pitch, self.body_yaw

        msg.StepLength = self.StepLength
        msg.LateralFraction = self.LateralFraction
        msg.YawRate = self.YawRate
        msg.ClearanceHeight = self.ClearanceHeight
        msg.PenetrationDepth = self.PenetrationDepth
        msg.SwingPeriod = self.Tswing
        msg.Ts_const = True
        msg.YawControlOn = 0.0

        # publish command
        self.pub_gait.publish(msg)
