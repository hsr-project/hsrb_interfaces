/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSR_KINEMATICS_HEAD_KINEMATICS_HPP__
#define HSR_KINEMATICS_HEAD_KINEMATICS_HPP__

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>


namespace hsr_kinematics {

bool CalculateHeadAngleToGazePoint(
    const Eigen::Translation3d& head_center_yaw_to_gaze_point,
    const Eigen::Affine3d& head_center_yaw_to_camera_frame,
    const Eigen::Affine3d& head_center_yaw_to_head_center_pitch,
    tmc_manipulation_types::JointState& dst_head_angle);

class HsrHeadKinematics {
 public:
  typedef std::shared_ptr<HsrHeadKinematics> Ptr;

  HsrHeadKinematics(const std::string& robot_model_config,
                    const std::vector<std::string>& head_joint_names);

  bool CalculateAngleToGazePoint(
      const Eigen::Translation3d& robot_base_to_gaze_point,
      const std::string& camera_frame_name,
      const tmc_manipulation_types::JointState& current_joint_state,
      tmc_manipulation_types::JointState& dst_head_angle);

 private:
  void SaturateToAngleLimit_(double angle,
                             const std::string& joint_name,
                             double& dst_angle);
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_model_;
  std::string pan_joint_name_;
  std::string tilt_joint_name_;
};
}  // namespace hsr_kinematics

#endif  // HSR_KINEMATICS_HEAD_KINEMATICS_HPP__
