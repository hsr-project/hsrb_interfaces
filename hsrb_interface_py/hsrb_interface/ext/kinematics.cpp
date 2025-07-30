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
/// @brief Python wrapper for the kinematics library
/// @brief Copyright (C) 2017 TOYOTA MOTOR CORPORATION
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <hsr_kinematics/head_kinematics.hpp>

namespace bp = boost::python;

namespace {
// Joint name of the head Yaw joint
const char* const kHeadYawJoint = "head_pan_joint";
// Joint name of the head pitch joint
const char* const kHeadPitchJoint = "head_tilt_joint";
}  // namespace

namespace hsrb_interface_plugin {

/// Kinematics interface, implemented only for calculating head joint angles to face any given coordinates
class KinematicsInterface {
 public:
  /// Constructor
  /// @param[in] robot_description Robot model
  explicit KinematicsInterface(const std::string& robot_description) {
    std::vector<std::string> head_joint_names;
    head_joint_names.push_back(kHeadYawJoint);
    head_joint_names.push_back(kHeadPitchJoint);
    head_kinematics_.reset(
        new hsr_kinematics::HsrHeadKinematics(robot_description,
                                              head_joint_names));
  }

  /// Calculates head joint angles to look at the given location
  /// @param[in] baselink_to_point Point coordinates in robot's base frame, [x, y, z]
  /// @param[in] camera_frame Frame name of the target camera
  /// @return boost::python::dict Head pan and tilt joint angles
  ///                             Returns an empty dictionary if calculation fails
  bp::dict CalculateAngles(const bp::list& baselink_to_point,
                           const std::string& camera_frame) {
    if (bp::len(baselink_to_point) != 3) {
      throw std::invalid_argument("Invalid input point.");
    }
    Eigen::Translation3d target(
        static_cast<double>(bp::extract<double>(baselink_to_point[0])),
        static_cast<double>(bp::extract<double>(baselink_to_point[1])),
        static_cast<double>(bp::extract<double>(baselink_to_point[2])));
    // The third argument current_joint_state can be omitted for HSR-B head configuration
    // head_joint_state will be clamped to its value if joint angle limits are exceeded
    tmc_manipulation_types::JointState head_joint_state;
    if (!head_kinematics_->CalculateAngleToGazePoint(
            target, camera_frame,
            tmc_manipulation_types::JointState(), head_joint_state)) {
      return bp::dict();
    }
    bp::dict result;
    for (uint32_t i = 0; i < head_joint_state.name.size(); ++i) {
      result[head_joint_state.name[i]] = head_joint_state.position[i];
    }
    return result;
  }

 private:
  hsr_kinematics::HsrHeadKinematics::Ptr head_kinematics_;
};

BOOST_PYTHON_MODULE(_extension) {
  bp::class_<hsrb_interface_plugin::KinematicsInterface>("KinematicsInterface", bp::init<std::string>())
      .def("calculate_gazing_angles", &hsrb_interface_plugin::KinematicsInterface::CalculateAngles);
}
}  // namespace hsrb_interface_plugin
