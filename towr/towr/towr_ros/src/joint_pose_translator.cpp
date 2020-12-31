#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <xpp_msgs/RobotStateJoint.h>


class JointPoseTranslator
{
public:
	JointPoseTranslator();

private:
   void StateCallback(const xpp_msgs::RobotStateJoint& msg);
	
  ros::NodeHandle nh_;

  ros::Subscriber joint_sub ;

  ros::Publisher joint_commands_publisher_;
};

JointPoseTranslator::JointPoseTranslator()
{
	joint_sub = nh_.subscribe("xpp/joint_spot_des", 1, &JointPoseTranslator::StateCallback, this);

	joint_commands_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 1);
}

void JointPoseTranslator::StateCallback(const xpp_msgs::RobotStateJoint& msg)
{

    trajectory_msgs::JointTrajectory joints_cmd_msg;
    joints_cmd_msg.header.stamp = ros::Time::now();
    joints_cmd_msg.joint_names = {"front_left_hip_x", "front_left_hip_y", "front_left_knee", "front_right_hip_x", "front_right_hip_y",
      "front_right_knee", "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee", "rear_right_hip_x",
      "rear_right_hip_y", "rear_right_knee"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(12);

    point.time_from_start = ros::Duration(1.0 / 60.0);
    for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = msg.joint_state.position.at(i);
        }
    joints_cmd_msg.points.push_back(point);
    joint_commands_publisher_.publish(joints_cmd_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_pose_translator");
  JointPoseTranslator joint_pose_translator;

  ros::spin();
}