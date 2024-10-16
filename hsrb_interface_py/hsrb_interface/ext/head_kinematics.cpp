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
#include "head_kinematics.hpp"

#include <float.h>
#include <string>
#include <vector>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

namespace {
const int32_t kHeadDof = 2;

bool CalculateHeadPitchAngleToGazePoint(
    const Eigen::Translation3d& head_center_pitch_to_gaze_point,
    const Eigen::Affine3d& head_center_pitch_to_camera_frame,
    double& dst_angle) {
  double distance_to_gaze_point =
      sqrt(pow(head_center_pitch_to_gaze_point.x(), 2.0) +
           pow(head_center_pitch_to_gaze_point.y(), 2.0) +
           pow(head_center_pitch_to_gaze_point.z(), 2.0));
  if (distance_to_gaze_point < DBL_MIN) {
    return false;
  }

  double distance_to_camera =
      sqrt(pow(head_center_pitch_to_camera_frame.translation().x(), 2.0) +
           pow(head_center_pitch_to_camera_frame.translation().y(), 2.0) +
           pow(head_center_pitch_to_camera_frame.translation().z(), 2.0));
  double theta_camera = atan2(
      head_center_pitch_to_camera_frame.translation().z(),
      sqrt(pow(head_center_pitch_to_camera_frame.translation().x(), 2.0) +
           pow(head_center_pitch_to_camera_frame.translation().y(), 2.0)));

  double theta_gaze_point = atan2(
      head_center_pitch_to_gaze_point.z(),
      sqrt(pow(head_center_pitch_to_gaze_point.x(), 2.0) +
           pow(head_center_pitch_to_gaze_point.y(), 2.0)));

  double temp_value = distance_to_camera /
      distance_to_gaze_point *
      sin(theta_camera);

  if (fabs(temp_value) > 1.0) {
    return false;
  }
  dst_angle = theta_gaze_point - asin(temp_value);
  return true;
}

bool CalculateHeadYawAngleToGazePoint(
    const Eigen::Translation3d& head_center_yaw_to_gaze_point,
    const Eigen::Affine3d& head_center_yaw_to_camera_frame,
    double& dst_angle) {

  Eigen::Translation3d gaze_point = head_center_yaw_to_gaze_point;
  Eigen::Affine3d camera_frame = head_center_yaw_to_camera_frame;

  double distance_to_gaze_point =
      sqrt(pow(gaze_point.x(), 2.0) +
           pow(gaze_point.y(), 2.0) +
           pow(gaze_point.z(), 2.0));

  if (distance_to_gaze_point < DBL_MIN) {
    return false;
  }

  double distance_to_camera =
      sqrt(pow(camera_frame.translation().x(), 2.0) +
           pow(camera_frame.translation().y(), 2.0) +
           pow(camera_frame.translation().z(), 2.0));
  double theta_camera = atan2(camera_frame.translation().y(),
                              camera_frame.translation().x());
  double theta_gaze_point = atan2(gaze_point.y(),
                                  gaze_point.x());

  double temp_value = distance_to_camera /
      distance_to_gaze_point *
      sin(theta_camera);

  if (fabs(temp_value) > 1.0) {
    return false;
  }
  dst_angle = theta_gaze_point - asin(temp_value);
  if ((head_center_yaw_to_gaze_point.x() < 0) &&
      (dst_angle > 0)) {
    dst_angle = - 2.0 * M_PI + dst_angle;
  }

  return true;
}
}  // namespace

namespace hsr_kinematics {

bool CalculateHeadAngleToGazePoint(
    const Eigen::Translation3d& head_center_yaw_to_gaze_point,
    const Eigen::Affine3d& head_center_yaw_to_camera_frame,
    const Eigen::Affine3d& head_center_yaw_to_head_center_pitch,
    const std::string& head_pan_joint_name,
    const std::string& head_tilt_joint_name,
    tmc_manipulation_types::JointState& dst_head_angle) {
  double yaw_angle = 0.0;
  if (!CalculateHeadYawAngleToGazePoint(head_center_yaw_to_gaze_point,
                                        head_center_yaw_to_camera_frame,
                                        yaw_angle)) {
    return false;
  }
  Eigen::Translation3d head_center_pitch_to_gaze_point(
      (head_center_yaw_to_head_center_pitch.inverse() *
       head_center_yaw_to_gaze_point).translation());
  Eigen::Affine3d head_center_pitch_to_camera_frame =
      head_center_yaw_to_head_center_pitch.inverse() *
      head_center_yaw_to_camera_frame;

  double pitch_angle = 0.0;
  if (!CalculateHeadPitchAngleToGazePoint(head_center_pitch_to_gaze_point,
                                head_center_pitch_to_camera_frame,
                                pitch_angle)) {
    return false;
  }

  dst_head_angle.name.resize(2);
  dst_head_angle.name[0] = head_pan_joint_name;
  dst_head_angle.name[1] = head_tilt_joint_name;
  dst_head_angle.position.resize(2);
  dst_head_angle.position[0] = yaw_angle;
  dst_head_angle.position[1] = pitch_angle;

  return true;
}

HsrHeadKinematics::HsrHeadKinematics(const std::string& robot_model_config,
                                     const std::vector<std::string>& head_joint_names) {
  if (head_joint_names.size() != kHeadDof) {
    throw std::invalid_argument("The size of head_joint_names is invalid.");
  }
  pan_joint_name_ = head_joint_names[0];
  tilt_joint_name_ = head_joint_names[1];
  robot_model_.reset(
      new tmc_robot_kinematics_model::PinocchioWrapper(robot_model_config));
}

bool HsrHeadKinematics::CalculateAngleToGazePoint(
    const Eigen::Translation3d& robot_base_to_gaze_point,
    const std::string& camera_frame_name,
    const tmc_manipulation_types::JointState& current_joint_state,
    tmc_manipulation_types::JointState& dst_head_angle) {
  robot_model_->SetNamedAngle(current_joint_state);
  tmc_manipulation_types::JointState zero_angle;
  zero_angle.name.push_back(pan_joint_name_);
  zero_angle.name.push_back(tilt_joint_name_);
  zero_angle.position.resize(2);
  zero_angle.position[0] = 0.0;
  zero_angle.position[1] = 0.0;
  robot_model_->SetNamedAngle(zero_angle);

  Eigen::Affine3d robot_base_to_head_yaw =
      robot_model_->GetObjectTransform(pan_joint_name_);
  Eigen::Translation3d head_yaw_to_gaze_point(
      (robot_base_to_head_yaw.inverse()
       * robot_base_to_gaze_point).translation());
  Eigen::Affine3d head_yaw_to_camera_frame =
      robot_model_->GetObjectRelativeTransform(
          pan_joint_name_, camera_frame_name);
  Eigen::Affine3d head_yaw_to_head_pitch =
      robot_model_->GetObjectRelativeTransform(pan_joint_name_, tilt_joint_name_);
  if (!CalculateHeadAngleToGazePoint(head_yaw_to_gaze_point,
                                     head_yaw_to_camera_frame,
                                     head_yaw_to_head_pitch,
                                     pan_joint_name_,
                                     tilt_joint_name_,
                                     dst_head_angle)) {
    return false;
  }
  SaturateToAngleLimit_(dst_head_angle.position[0],
                        pan_joint_name_,
                        dst_head_angle.position[0]);
  SaturateToAngleLimit_(dst_head_angle.position[1],
                        tilt_joint_name_,
                        dst_head_angle.position[1]);
  return true;
}

void HsrHeadKinematics::SaturateToAngleLimit_(double angle,
                                              const std::string& joint_name,
                                              double& dst_angle) {
  tmc_manipulation_types::NameSeq use_joints;
  use_joints.push_back(joint_name);
  Eigen::VectorXd min;
  Eigen::VectorXd max;
  robot_model_->GetMinMax(use_joints, min, max);
  if (angle > max[0]) {
    dst_angle = max[0];
  } else if (angle < min[0]) {
    dst_angle = min[0];
  } else {
    dst_angle = angle;
  }
}
}  // namespace hsr_kinematics
