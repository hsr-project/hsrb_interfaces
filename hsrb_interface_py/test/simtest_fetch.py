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

"""Test fetching a box with marker."""
import math

from hsrb_interface import _testing as testing
from hsrb_interface import geometry

BOX_ID = 27


class FetchTest(testing.HsrbInterfaceTest):
    """Test simple object manipulation operations."""

    def test_fetch_box_with_marker(self):
        """The robot should grasp a box with marker successfully."""
        self.whole_body.move_to_go()
        desk_pose = geometry.pose(x=1.70, y=1.85, z=0.36)
        desk_id = self.collision_world.add_box(x=1.0, y=1.1, z=0.72,
                                               pose=desk_pose,
                                               timeout=3.0)
        self.assertIsNotNone(desk_id)
        self.omni_base.go(0.5, 1.5, 0.0, relative=True)
        self.whole_body.move_to_go()
        relative_box_pose = geometry.pose(0.85, 0.02, 0.7, ek=math.pi / 2.0)
        self.expect_detected_object(BOX_ID, relative_box_pose,
                                    pos_delta=0.05, ori_delta=math.radians(5),
                                    frame='base_footprint')
        box = self.marker.get_object_by_id(BOX_ID)

        box_pose = box.get_pose(ref_frame_id='base_footprint')
        hand_pose = geometry.pose(box_pose[0].x - 0.15,
                                  box_pose[0].y - 0.02,
                                  box_pose[0].z + 0.10,
                                  ej=-math.pi / 2.0, ek=math.pi)
        self.whole_body.move_to_neutral()
        self.gripper.command(1.0)
        self.whole_body.collision_world = self.collision_world
        self.whole_body.move_end_effector_pose(hand_pose, 'base_footprint')
        self.collision_world.remove((box.id, 'x'))
        self.gripper.command(0.0)
        self.whole_body.move_end_effector_by_line((1, 0, 0), 0.1,
                                                  'hand_palm_link')
        self.whole_body.move_end_effector_by_line((0, 0, -1), 0.5,
                                                  'hand_palm_link')
        self.whole_body.move_to_neutral()
        self.expect_gripper_to_grasp(delta=math.radians(1))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_fetch', FetchTest)
