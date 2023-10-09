#!/usr/bin/env python
# vim: fileencoding=utf-8 :
# Copyright (c) 2023 TOYOTA MOTOR CORPORATION
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

"""Test gripper interface in Gazebo simlulator."""

from hsrb_interface import _testing as testing


class GripperTest(testing.HsrbInterfaceTest):
    """Test cases for gripper interface."""

    def test_command_by_position(self):
        """The gripper should move to given position."""
        self.whole_body.move_to_neutral()
        self.expect_joints_reach_goals(self.EXPECTED_NEUTRAL, 0.01)

        self.gripper.command(1.0)
        self.expect_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        self.gripper.command(0.0)
        self.expect_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

    def test_command_to_grasp(self):
        """The gripper should move to given angle."""
        self.whole_body.move_to_neutral()

        self.gripper.command(1.0)
        self.expect_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        self.gripper.grasp(-0.01)
        self.expect_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_gripper', GripperTest)