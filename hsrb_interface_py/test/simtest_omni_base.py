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

"""Testing autonomous navigation interface in Gazebo simulator.

A tf frame 'my_frame' is published as::

    '2.0 0.0 0.5 3.14 -1.57 0.0 map my_frame 10'

Its static_transform_publisher is launched in .test file.
"""
import math

from hsrb_interface import _testing as testing
from hsrb_interface import geometry
from tf import transformations
import tf2_geometry_msgs
assert tf2_geometry_msgs  # silence flake8


class OmniBaseTest(testing.HsrbInterfaceTest):
    """Test cases omni_base interface."""

    def test_simple_move(self):
        """Short distance movement."""
        self.whole_body.move_to_go()
        self.omni_base.go(0, 0, 0, self.BASE_MOVE_TIME_TOLERANCE,
                          relative=False)
        self.omni_base.go(0.1, 0.0, 0.0, self.BASE_MOVE_TIME_TOLERANCE,
                          relative=True)
        self.expect_base_reach_goal(geometry.pose(0.1, 0.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

        self.omni_base.go(-0.1, 0.0, 0.0, self.BASE_MOVE_TIME_TOLERANCE,
                          relative=True)
        self.expect_base_reach_goal(geometry.pose(0.0, 0.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

    def test_get_current_position(self):
        """It should get current position correctly."""
        self.whole_body.move_to_go()
        self.omni_base.go(0, 0, 0, self.BASE_MOVE_TIME_TOLERANCE)

        pose2d = self.omni_base.pose
        pose3d = self.omni_base.get_pose('map')
        self.assertAlmostEqual(pose2d[0], pose3d[0].x)
        self.assertAlmostEqual(pose2d[1], pose3d[0].y)
        roll, pitch, yaw = transformations.euler_from_quaternion(pose3d[1])
        self.assertAlmostEqual(0, roll)
        self.assertAlmostEqual(0, pitch)
        self.assertAlmostEqual(pose2d[2], yaw)

    def test_move_globally(self):
        """The robot should move to global position."""
        self.whole_body.move_to_go()
        self.omni_base.go(6.0, 4.0, 0.0, 300.0, relative=False)
        self.expect_base_reach_goal(geometry.pose(6.0, 4.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

        self.omni_base.go(0.0, 0.0, 0.0, 300.0, relative=False)
        self.expect_base_reach_goal(geometry.pose(0.0, 0.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

    def test_move_relatively(self):
        """The robot should move to relative position."""
        self.whole_body.move_to_go()

        self.omni_base.go(0.0, 0.0, 0.0, 300.0, relative=False)
        self.expect_base_reach_goal(geometry.pose(0.0, 0.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

        self.omni_base.go(0.0, 1.0, 0.0, 300.0, relative=True)
        self.expect_base_reach_goal(geometry.pose(0.0, 1.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

        self.omni_base.go(0.0, -1.0, 0.0, 300.0, relative=True)
        self.expect_base_reach_goal(geometry.pose(0.0, 0.0, 0.0), 'map',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

    def test_move_using_tf(self):
        """Relative move using a tf frame 'my_frame'."""
        self.whole_body.move_to_go()

        goal = geometry.pose(ej=math.pi / 2.0)
        self.omni_base.move(goal, 100.0, ref_frame_id='my_frame')
        self.expect_base_reach_goal(goal, frame='my_frame',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

        goal = geometry.pose(z=-0.5, ej=math.pi / 2.0)
        self.omni_base.move(goal, 100.0, ref_frame_id='my_frame')
        self.expect_base_reach_goal(goal, frame='my_frame',
                                    pos_delta=0.05, ori_delta=0.05,
                                    timeout=self.BASE_MOVE_TIME_TOLERANCE)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_omni_base', OmniBaseTest)
