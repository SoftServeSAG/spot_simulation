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

#include <xpp_spot/spotleg_inverse_kinematics.h>
#include "ros/ros.h"

#ifdef __unix__
    #include <cmath>
    using namespace std;
#endif

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>

namespace xpp {


SpotlegInverseKinematics::Vector3d
SpotlegInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B, BodyLR bend, BodyFB fb) const
{
  double hip_joint, upper_leg_joint, lower_leg_joint;

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

    xr = ee_pos_B;

  double l0 = 0.0f;
  if (bend==Left) {
    l0 = left_hip_y_y;
    xr[Y] = xr[Y];
    }
  else // backward
  {
    l0 = -right_hip_y_y;
    xr[Y] = -xr[Y];
    }

  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg

  if (fb==Front)
    xr[X] = xr[X];
  else // backward
    xr[X] = -xr[X];

//  cout << "xr[X] = " << xr[X] << endl;
//  cout << "xr[Y] = " << xr[Y] << endl;
//  cout << "xr[Z] = " << xr[Z] << endl;

  double x = xr[X];
  double y = xr[Y];
  double z = xr[Z];

  // compute the HAA angle
  hip_joint = -(atanf(y / z) - ((M_PI/2) - acosf(-l0 / sqrtf(pow(y, 2) + pow(z, 2)))));

  // rotate into the HFE coordinate system (rot around X)
  R << 1, 0, 0, 0, cos(-hip_joint), -sin(-hip_joint), 0, sin(-hip_joint), cos(-hip_joint);
  xr = (R * xr).eval();


  // translate into the HFE coordinate system (along Z axis)
  xr += hfe_to_haa_z;  //distance of HFE to HAA in z direction

  x = xr[X];
  y = xr[Y];
  z = xr[Z];

  //source: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
  lower_leg_joint = -acos((pow(xr[Z], 2) + pow(xr[X], 2) - pow(length_thigh ,2) - pow(length_shank ,2)) / (2 * length_thigh * length_shank));
  upper_leg_joint = (atan(xr[X] / xr[Z]) - atan( (length_shank * sin(lower_leg_joint)) / (length_thigh + (length_shank * cos(lower_leg_joint)))));

//  if(upper_leg_joint < 0)
//  {
//    upper_leg_joint = upper_leg_joint +  M_PI;
//  }

   EnforceLimits(hip_joint, HAA);
   EnforceLimits(upper_leg_joint, HFE);
   EnforceLimits(lower_leg_joint, KFE);

   if(isnan(hip_joint) || isnan(upper_leg_joint) || isnan(lower_leg_joint))
    ROS_INFO("hip_joint : [%g]\n upper_leg_joint : [%g]\n q_coude : [%g]\n", hip_joint, upper_leg_joint, lower_leg_joint);
  else
  {
    return Vector3d(hip_joint, upper_leg_joint, lower_leg_joint);
  }
}


void
SpotlegInverseKinematics::EnforceLimits (double& val, SpotJointID joint) const
{
  // totally exaggerated joint angle limits
  const static double haa_min = -0.7854;
  const static double haa_max =  0.7854;

  const static double hfe_min = -0.8988;
  const static double hfe_max =  2.2951;

  const static double kfe_min = -2.7929;
  const static double kfe_max =  0.254801;

  // reduced joint angles for optimization
  static const std::map<SpotJointID, double> max_range {
    {HAA, haa_max},
    {HFE, hfe_max},
    {KFE, kfe_max}
  };

  // reduced joint angles for optimization
  static const std::map<SpotJointID, double> min_range {
    {HAA, haa_min},
    {HFE, hfe_min},
    {KFE, kfe_min}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

} /* namespace xpp */
