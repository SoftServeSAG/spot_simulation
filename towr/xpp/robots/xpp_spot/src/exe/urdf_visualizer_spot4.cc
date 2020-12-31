/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_spot/inverse_kinematics_spot4.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "spot_urdf_visualizer");

  const std::string joint_desired_spot = "xpp/joint_spot_des";

  auto spot_ik = std::make_shared<InverseKinematicsSpot4>();
  CartesianJointConverter inv_kin_converter(spot_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_spot);

  // urdf joint names
  int n_ee = spot_ik->GetEECount();
  int n_j  = SpotlegJointCount;
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee*n_j);
  joint_names.at(n_j*LF + HAA) = "front_left_hip_x";
  joint_names.at(n_j*LF + HFE) = "front_left_hip_y";
  joint_names.at(n_j*LF + KFE) = "front_left_knee";
  joint_names.at(n_j*RF + HAA) = "front_right_hip_x";
  joint_names.at(n_j*RF + HFE) = "front_right_hip_y";
  joint_names.at(n_j*RF + KFE) = "front_right_knee";
  joint_names.at(n_j*LH + HAA) = "rear_left_hip_x";
  joint_names.at(n_j*LH + HFE) = "rear_left_hip_y";
  joint_names.at(n_j*LH + KFE) = "rear_left_knee";
  joint_names.at(n_j*RH + HAA) = "rear_right_hip_x";
  joint_names.at(n_j*RH + HFE) = "rear_right_hip_y";
  joint_names.at(n_j*RH + KFE) = "rear_right_knee";

  std::string urdf = "spot_description";
  UrdfVisualizer spot_desired(urdf, joint_names, "body", "world",
			     joint_desired_spot, "spot_des");

  ::ros::spin();

  return 1;
}

