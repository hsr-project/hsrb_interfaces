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

"""Test collision_world object."""

import math

from hsrb_interface import _testing as testing
from hsrb_interface import geometry
import rospy
from tmc_manipulation_msgs.msg import CollisionObject

COLLISION_ENVIRONMENT_TIMEOUT = 30.0


class CollisionWorldTest(testing.HsrbInterfaceTest):
    """Test for CollisionWorld interface"""

    def setUp(self):
        """Called before calling each test method

        XXX: Waiting collision_environment start subscription.
             We need more reliable method to wait until the node start up.
        """
        known_object_pub = rospy.Publisher('known_object', CollisionObject,
                                           queue_size=1)
        start = rospy.Time.now()
        while known_object_pub.get_num_connections() == 0:
            now = rospy.Time.now()
            if (now - start).to_sec() < COLLISION_ENVIRONMENT_TIMEOUT:
                rospy.sleep(0.1)
            else:
                self.fail("Faild to connect to collision environemnt.")

    def tearDown(self):
        """Remove all objects"""
        self.collision_world.remove_all()

    def test_add_and_remove_objects(self):
        """It should be able to create obejects in collision world."""
        box_pose = geometry.pose(x=1.0, z=0.15)
        box_id = self.collision_world.add_box(x=0.3, y=0.3, z=0.3,
                                              pose=box_pose,
                                              timeout=3.0)
        self.assertIsNotNone(box_id)

        sphere_pose = geometry.pose(x=1.0, y=1.0, z=0.5)
        sphere_id = self.collision_world.add_sphere(radius=0.3,
                                                    pose=sphere_pose,
                                                    timeout=3.0)
        self.assertIsNotNone(sphere_id)

        cylinder_pose = geometry.pose(x=1.0, y=-1.0, z=0.5)
        cylinder_id = self.collision_world.add_cylinder(radius=0.1, length=1.0,
                                                        pose=cylinder_pose,
                                                        timeout=3.0)
        self.assertIsNotNone(cylinder_id)

        mesh_id = self.collision_world.add_mesh(
            filename='pacakge:///hsrb_interface_py/test/chair.stl',
            pose=geometry.pose(x=2.0, ei=math.radians(90)),
            frame_id='map', name='chair'
        )
        self.assertIsNotNone(mesh_id)

        snapshot = self.collision_world.snapshot()
        objects = [o.id.object_id for o in snapshot.known_objects]
        self.assertIn(box_id[0], objects)
        self.assertIn(sphere_id[0], objects)
        self.assertIn(cylinder_id[0], objects)
        self.assertIn(mesh_id[0], objects)

        self.collision_world.remove_all()

        snapshot = self.collision_world.snapshot()
        objects = [o.id.object_id for o in snapshot.known_objects]
        self.assertNotIn(box_id[0], objects)
        self.assertNotIn(sphere_id[0], objects)
        self.assertNotIn(cylinder_id[0], objects)
        self.assertNotIn(mesh_id[0], objects)

    def test_remove_single_object(self):
        """It should remove only specified obeject from collision world."""
        box_pose = geometry.pose(x=1.0, z=0.15)
        box_id = self.collision_world.add_box(x=0.3, y=0.3, z=0.3,
                                              pose=box_pose,
                                              timeout=3.0)
        self.assertIsNotNone(box_id)

        pose = geometry.pose(x=1.0, y=1.0, z=0.5)
        sphere_id = self.collision_world.add_sphere(radius=0.3,
                                                    pose=pose,
                                                    timeout=3.0)
        self.assertIsNotNone(sphere_id)

        self.collision_world.remove(sphere_id)

        snapshot = self.collision_world.snapshot()
        objects = [o.id.object_id for o in snapshot.known_objects]
        self.assertIn(box_id[0], objects)
        self.assertNotIn(sphere_id[0], objects)

        self.collision_world.remove_all()

    def test_collision_avoidance(self):
        """Testing collision check works correctly."""
        pose = geometry.pose(x=0.6, y=0.0, z=0.65)
        box_id = self.collision_world.add_box(x=0.3, y=0.3, z=0.3,
                                              pose=pose,
                                              timeout=3.0)
        self.assertIsNotNone(box_id)

        self.whole_body.move_to_neutral()
        self.whole_body.collision_world = self.collision_world

        goal = geometry.pose(0.7, 0.0, 0.25, ej=math.pi / 2.0)
        self.whole_body.move_end_effector_pose(goal, 'map')
        self.expect_hand_reach_goal(goal, frame='map', pos_delta=0.05,
                                    ori_delta=math.radians(2.0))
        self.whole_body.collision_world = None


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_collision',
                   CollisionWorldTest)
