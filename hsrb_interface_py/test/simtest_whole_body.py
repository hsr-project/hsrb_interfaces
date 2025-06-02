#!/usr/bin/env python
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
"""Testing motion planning interface in Gazebo simulator."""

import unittest

import _testing as testing


class WholeBodyTest(testing.HsrbInterfaceTest):
    """Test cases for whole_body object."""

    def test_move_to_joint_positions(self):
        """Driving each joint."""
        self.whole_body.move_to_neutral()

        self.assertListEqual(self.JOINT_NAMES, sorted(self.whole_body.joint_names))

        self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})
        expected_pose = {'arm_lift_joint': 0.2}
        self.expect_joints_reach_goals(expected_pose, delta=0.01)

        self.whole_body.move_to_joint_positions({'head_pan_joint': 0.4,
                                                 'head_tilt_joint': -0.2})
        expected_pose = {'head_pan_joint': 0.4,
                         'head_tilt_joint': -0.2}
        self.expect_joints_reach_goals(expected_pose, delta=0.01)

    def test_move_to_joint_positions_multiple_targets(self):
        """Driving each joint."""
        self.whole_body.move_to_neutral()

        self.assertListEqual(self.JOINT_NAMES, sorted(self.whole_body.joint_names))

        self.whole_body.move_to_joint_positions_multiple_targets(
            ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint'],
            [[0.2, -0.3, -1.57]]
        )
        expected_pose = {
            'arm_lift_joint': 0.2,
            'arm_flex_joint': -0.3,
            'arm_roll_joint': -1.57
        }
        self.expect_joints_reach_goals(expected_pose, delta=0.01)


if __name__ == '__main__':
    unittest.main()
