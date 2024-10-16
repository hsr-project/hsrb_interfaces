'''
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
'''
"""Unittest for collision_world module"""
import math
import os
import sys

from unittest.mock import ANY
from unittest.mock import MagicMock
from unittest.mock import patch
from unittest.mock import PropertyMock

import _testing as testing
import hsrb_interface
from hsrb_interface import geometry

import hsrb_interface.collision_world
import hsrb_interface.exceptions
from hsrb_interface.robot import Robot

from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningSceneWorld
from moveit_msgs.msg import RobotState

from nose.tools import eq_
from nose.tools import raises

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

import rclpy.time

from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class CollisionWorldTest(testing.RosMockTestCase):

    def create(self):
        """Create a CollisionWorld instance"""
        self.get_frame_mock.return_value = 'map'
        self.get_entry_mock.return_value = {
            "class": ["collision_world", "CollisionWorld"],
            "control_topic": "/collision_environment_server/collision_object",
            "environment_topic": "/collision_environment_server/environment",
            "trans_env_topic": "/collision_environment_server/transformed_environment",
            "set_frame_service": "/collision_environment_server/set_parameters",
            "attaching_topic": "/attached_object_publisher/attaching_object_name",
            "add_attaching_topic": "/attached_object_publisher/attaching_object_info",
            "releasing_topic": "/attached_object_publisher/releasing_object_name",
            "attached_info_topic": "/attached_object_publisher/attached_object",
            "end_effector_frames": [
                "hand_palm_link",
                "hand_l_finger_vacuum_frame"
            ]
        }

        patcher = patch.object(Robot, "_connection")
        self.node_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.publisher_mock = self.node_mock.create_publisher
        self.service_client_mock = self.node_mock.create_client

        patcher = patch("rclpy.spin_until_future_complete")
        self.spin_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("rclpy.ok")
        self.rclpy_ok_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.rclpy_ok_mock = True

        patcher = patch("rclpy.spin_once")
        self.rclpy_spin_once_mock = patcher.start()
        self.addCleanup(patcher.stop)

        return hsrb_interface.collision_world.CollisionWorld('test')

    def create_test_environment(self):
        test_environment = PlanningSceneWorld()
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.1, 0.1, 0.1]
        collision_object = CollisionObject()
        collision_object.id = 'box'
        pose = geometry.pose(1.0, 2.0, 3.0)
        collision_object.primitive_poses.append(geometry.tuples_to_pose(pose))
        test_environment.collision_objects.append(collision_object)

        return test_environment

    def test_creation(self):
        """Test simple use case of MobileBase class"""
        self.create()

        self.node_mock.create_publisher.assert_any_call(CollisionObject,
                                                        '/collision_environment_server/collision_object',
                                                        ANY)
        self.node_mock.create_publisher.assert_any_call(String,
                                                        '/attached_object_publisher/attaching_object_name',
                                                        ANY)
        self.node_mock.create_publisher.assert_any_call(AttachedCollisionObject,
                                                        '/attached_object_publisher/attaching_object_info',
                                                        ANY)
        self.node_mock.create_publisher.assert_any_call(String,
                                                        '/attached_object_publisher/releasing_object_name',
                                                        ANY)
        self.caching_sub_mock.assert_any_call('/collision_environment_server/environment',
                                              PlanningSceneWorld,
                                              default=PlanningSceneWorld())
        self.caching_sub_mock.assert_any_call('/collision_environment_server/transformed_environment',
                                              PlanningSceneWorld,
                                              default=PlanningSceneWorld())
        self.caching_sub_mock.assert_any_call('/attached_object_publisher/attached_object',
                                              RobotState,
                                              default=RobotState())

    def test_snapshot(self):
        collision_world = self.create()
        sub_mock = MagicMock()
        collision_world._trans_env_sub = sub_mock

        test_environment = self.create_test_environment()
        data_mock = PropertyMock(return_value=test_environment)
        type(sub_mock).data = data_mock

        param_service_client_mock = self.service_client_mock.return_value
        plan_result_mock = MagicMock()
        response = SetParameters.Response()
        result = SetParametersResult()
        result.successful = True
        response.results.append(result)
        error_code_mock = PropertyMock(return_value=response)
        type(plan_result_mock).val = error_code_mock
        param_service_client_mock.call_async.return_value = plan_result_mock

        snapshot = collision_world.snapshot()
        eq_(snapshot, test_environment)
        self.service_client_mock.assert_called_with(
            SetParameters,
            '/collision_environment_server/set_parameters')
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'origin_frame_id'
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = 'map'
        req.parameters.append(param)

        param_service_client_mock.call_async.assert_called_with(req)

    def test_add_box(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'box'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(1.0, 2.0, 3.0)
        id = collision_world.add_box(pose=pose)
        eq_(id, 'box')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.1, 0.1, 0.1]
        msg = CollisionObject()
        msg.id = 'box'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = 'map'
        msg.operation = CollisionObject.ADD
        msg.primitives.append(shape)
        msg.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_attached_box(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._attach_info_sub = sub_mock

        test_state = RobotState()
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.object.id = 'box'
        test_state.attached_collision_objects.append(attached_collision_object)
        data_mock = PropertyMock(side_effect=[RobotState(),
                                              test_state,
                                              test_state])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(z=0.025)
        id = collision_world.add_attached_box(x=0.05, y=0.05, z=0.05, pose=pose)
        eq_(id, 'box')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.05, 0.05, 0.05]
        msg = AttachedCollisionObject()
        msg.link_name = 'hand_palm_link'
        msg.object.id = 'box'
        msg.object.header.stamp = rclpy.time.Time().to_msg()
        msg.object.header.frame_id = 'hand_palm_link'
        msg.object.operation = CollisionObject.ADD
        msg.object.primitives.append(shape)
        msg.object.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

        attached_objects = collision_world.attached_objects
        eq_(len(attached_objects), 1)
        eq_(attached_objects[0].object.id, 'box')

    @raises(ValueError)
    def test_add_attached_box_bad_frame_id(self):
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        pose = geometry.pose(z=0.025)
        collision_world.add_attached_box(x=0.05, y=0.05, z=0.05, pose=pose, frame_id='arm_lift_joint')

    def test_add_boxes(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'box'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        poses = [geometry.pose(1.0, 2.0, 3.0),
                 geometry.pose(4.0, 5.0, 6.0),
                 geometry.pose(7.0, 8.0, 9.0)]
        id = collision_world.add_box(pose=poses)
        eq_(id, 'box')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [0.1, 0.1, 0.1]
        msg = CollisionObject()
        msg.id = 'box'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = 'map'
        msg.operation = CollisionObject.ADD
        for pose in poses:
            msg.primitives.append(shape)
            msg.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_sphere(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'sphere'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(1.0, 2.0, 3.0)
        id = collision_world.add_sphere(pose=pose)
        eq_(id, 'sphere')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [0.1]
        msg = CollisionObject()
        msg.id = 'sphere'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = 'map'
        msg.operation = CollisionObject.ADD
        msg.primitives.append(shape)
        msg.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_attached_sphere(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._attach_info_sub = sub_mock

        test_state = RobotState()
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.object.id = 'sphere'
        test_state.attached_collision_objects.append(attached_collision_object)
        data_mock = PropertyMock(side_effect=[RobotState(),
                                              test_state,
                                              test_state])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(z=0.025)
        id = collision_world.add_attached_sphere(radius=0.03, pose=pose)
        eq_(id, 'sphere')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [0.03]
        msg = AttachedCollisionObject()
        msg.link_name = 'hand_palm_link'
        msg.object.id = 'sphere'
        msg.object.header.stamp = rclpy.time.Time().to_msg()
        msg.object.header.frame_id = 'hand_palm_link'
        msg.object.operation = CollisionObject.ADD
        msg.object.primitives.append(shape)
        msg.object.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

        attached_objects = collision_world.attached_objects
        eq_(len(attached_objects), 1)
        eq_(attached_objects[0].object.id, 'sphere')

    @raises(ValueError)
    def test_add_attached_sphere_bad_frame_id(self):
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        collision_world.add_attached_sphere(radius=0.03, frame_id='arm_lift_joint')

    def test_add_spheres(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'sphere'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        poses = [geometry.pose(1.0, 2.0, 3.0),
                 geometry.pose(4.0, 5.0, 6.0),
                 geometry.pose(7.0, 8.0, 9.0)]
        id = collision_world.add_sphere(pose=poses)
        eq_(id, 'sphere')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [0.1]
        msg = CollisionObject()
        msg.id = 'sphere'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = 'map'
        msg.operation = CollisionObject.ADD
        for pose in poses:
            msg.primitives.append(shape)
            msg.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_cylinder(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'cylinder'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(1.0, 2.0, 3.0)
        id = collision_world.add_cylinder(pose=pose)
        eq_(id, 'cylinder')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [0.1, 0.1]
        msg = CollisionObject()
        msg.id = 'cylinder'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = 'map'
        msg.operation = CollisionObject.ADD
        msg.primitives.append(shape)
        msg.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_attached_cylinder(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._attach_info_sub = sub_mock

        test_state = RobotState()
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.object.id = 'cylinder'
        test_state.attached_collision_objects.append(attached_collision_object)
        data_mock = PropertyMock(side_effect=[RobotState(),
                                              test_state,
                                              test_state])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(z=0.025, ej=math.radians(90))
        id = collision_world.add_attached_cylinder(radius=0.025, length=1.0, pose=pose)
        eq_(id, 'cylinder')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [1.0, 0.025]
        msg = AttachedCollisionObject()
        msg.link_name = 'hand_palm_link'
        msg.object.id = 'cylinder'
        msg.object.header.stamp = rclpy.time.Time().to_msg()
        msg.object.header.frame_id = 'hand_palm_link'
        msg.object.operation = CollisionObject.ADD
        msg.object.primitives.append(shape)
        msg.object.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

        attached_objects = collision_world.attached_objects
        eq_(len(attached_objects), 1)
        eq_(attached_objects[0].object.id, 'cylinder')

    @raises(ValueError)
    def test_add_attached_cylinder_bad_frame_id(self):
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        collision_world.add_attached_cylinder(radius=0.025, length=1.0, frame_id='arm_lift_joint')

    def test_add_cylinders(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'cylinder'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        poses = [geometry.pose(1.0, 2.0, 3.0),
                 geometry.pose(4.0, 5.0, 6.0),
                 geometry.pose(7.0, 8.0, 9.0)]
        id = collision_world.add_cylinder(pose=poses)
        eq_(id, 'cylinder')
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [0.1, 0.1]
        msg = CollisionObject()
        msg.id = 'cylinder'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = 'map'
        msg.operation = CollisionObject.ADD
        for pose in poses:
            msg.primitives.append(shape)
            msg.primitive_poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    # def test_add_mesh_success(self):
    #     pub_mock = self.publisher_mock.return_value
    #     sub_mock = MagicMock()
    #     collision_world = self.create()
    #     collision_world._known_obj_ids_sub = sub_mock

    #     next_id = collision_world.next_object_id
    #     known_ids = ObjectIdentifierArray()
    #     known_id = ObjectIdentifier(object_id=next_id, name='mesh')
    #     known_ids.object_ids.append(known_id)
    #     data_mock = PropertyMock(side_effect=[ObjectIdentifierArray(),
    #                                           known_ids])
    #     # PropertyMock must be attached to class
    #     type(sub_mock).data = data_mock

    #     pose = geometry.pose(1, 2, 3)
    #     id, name = collision_world.add_mesh('hoge.stl', pose=pose)
    #     eq_(id, next_id)
    #     eq_(name, 'mesh')
    #     shape = Shape()
    #     shape.type = Shape.MESH
    #     shape.stl_file_name = 'hoge.stl'
    #     msg = CollisionObject()
    #     msg.id = ObjectIdentifier(object_id=next_id, name='mesh')
    #     msg.header.stamp = ANY
    #     msg.header.frame_id = 'map'
    #     msg.operation.operation = CollisionObjectOperation.ADD
    #     msg.shapes.append(shape)
    #     msg.poses.append(geometry.tuples_to_pose(pose))
    #     pub_mock.publish.assert_called_with(msg)

    # def test_add_meshes_success(self):
    #     pub_mock = self.publisher_mock.return_value
    #     sub_mock = MagicMock()
    #     collision_world = self.create()
    #     collision_world._known_obj_ids_sub = sub_mock

    #     next_id = collision_world.next_object_id
    #     known_ids = ObjectIdentifierArray()
    #     known_id = ObjectIdentifier(object_id=next_id, name='mesh')
    #     known_ids.object_ids.append(known_id)
    #     data_mock = PropertyMock(side_effect=[ObjectIdentifierArray(),
    #                                           known_ids])
    #     # PropertyMock must be attached to class
    #     type(sub_mock).data = data_mock

    #     poses = [geometry.pose(1, 2, 3), geometry.pose(4, 5, 6), geometry.pose(7, 8, 9)]
    #     id, name = collision_world.add_mesh('hoge.stl', pose=poses)
    #     eq_(id, next_id)
    #     eq_(name, 'mesh')
    #     shape = Shape()
    #     shape.type = Shape.MESH
    #     shape.stl_file_name = 'hoge.stl'
    #     msg = CollisionObject()
    #     msg.id = ObjectIdentifier(object_id=next_id, name='mesh')
    #     msg.header.stamp = ANY
    #     msg.header.frame_id = 'map'
    #     msg.operation.operation = CollisionObjectOperation.ADD
    #     for pose in poses:
    #         msg.shapes.append(shape)
    #         msg.poses.append(geometry.tuples_to_pose(pose))
    #     pub_mock.publish.assert_called_with(msg)

    def test_attach(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value

        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock
        attach_sub_mock = MagicMock()
        collision_world._attach_info_sub = attach_sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'box'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[test_world,
                                              test_world])
        type(sub_mock).data = data_mock

        test_state = RobotState()
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.object.id = 'box'
        test_state.attached_collision_objects.append(attached_collision_object)
        data_mock = PropertyMock(side_effect=[test_state])
        type(attach_sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        id = collision_world.attach('box')
        eq_(id, 'box')
        msg = String()
        msg.data = 'box'
        pub_mock.publish.assert_called_with(msg)

    def test_release(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value

        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'box'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[test_world,
                                              test_world])
        type(sub_mock).data = data_mock

        self.assertTrue(collision_world.release('box'))
        msg = String()
        msg.data = 'box'
        pub_mock.publish.assert_called_with(msg)

    def test_release_all(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value

        sub_mock = MagicMock()
        collision_world._attach_info_sub = sub_mock

        data_mock = PropertyMock(side_effect=[RobotState(),
                                              RobotState(),
                                              RobotState()])
        type(sub_mock).data = data_mock

        self.assertTrue(collision_world.release_all())
        msg = AttachedCollisionObject()
        msg.object = CollisionObject()
        msg.object.operation = CollisionObject.REMOVE
        msg.link_name = 'hand_palm_link'
        pub_mock.publish.assert_called_with(msg)

        attached_objects = collision_world.attached_objects
        eq_(len(attached_objects), 0)

    def test_remove(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'answer'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        collision_world.remove('answer')

        msg = String()
        msg.data = 'answer'
        pub_mock.publish.assert_any_call(msg)

        msg = CollisionObject()
        msg.id = 'answer'
        msg.operation = CollisionObject.REMOVE
        pub_mock.publish.assert_any_call(msg)

    def test_remove_attached_object(self):
        collision_world = self.create()
        environment_sub_mock = MagicMock()
        collision_world._environment_sub = environment_sub_mock
        attach_info_sub_mock = MagicMock()
        collision_world._attach_info_sub = attach_info_sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'box'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[PlanningSceneWorld(),
                                              test_world])
        type(environment_sub_mock).data = data_mock

        test_state_1 = RobotState()
        test_state_2 = RobotState()
        attached_collision_object_1 = AttachedCollisionObject()
        attached_collision_object_1.object.id = 'box'
        attached_collision_object_2 = AttachedCollisionObject()
        attached_collision_object_2.object.id = 'sphere'
        test_state_1.attached_collision_objects.append(attached_collision_object_1)
        test_state_1.attached_collision_objects.append(attached_collision_object_2)
        test_state_2.attached_collision_objects.append(attached_collision_object_2)
        data_mock = PropertyMock(side_effect=[RobotState(),
                                              test_state_1,
                                              test_state_1,
                                              test_state_1,
                                              test_state_2,
                                              test_state_2])
        type(attach_info_sub_mock).data = data_mock

        get_clock_mock = self.node_mock.get_clock.return_value
        get_clock_mock.now.return_value = rclpy.time.Time()

        pose = geometry.pose(z=0.025)
        box_id = collision_world.add_attached_box(x=0.05, y=0.05, z=0.05, pose=pose)
        eq_(box_id, 'box')

        pose = geometry.pose(z=0.025)
        sphere_id = collision_world.add_attached_sphere(radius=0.03, pose=pose)
        eq_(sphere_id, 'sphere')

        attached_objects = collision_world.attached_objects
        eq_(len(attached_objects), 2)
        eq_(attached_objects[0].object.id, 'box')
        eq_(attached_objects[1].object.id, 'sphere')

        collision_world.remove(box_id)

        attached_objects = collision_world.attached_objects
        eq_(len(attached_objects), 1)
        eq_(attached_objects[0].object.id, 'sphere')

    def test_remove_all(self):
        collision_world = self.create()
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world._environment_sub = sub_mock
        attach_sub_mock = MagicMock()
        collision_world._attach_info_sub = attach_sub_mock

        test_world = PlanningSceneWorld()
        collision_object = CollisionObject()
        collision_object.id = 'chair'
        test_world.collision_objects.append(collision_object)
        data_mock = PropertyMock(side_effect=[test_world,
                                              test_world,
                                              PlanningSceneWorld()])
        type(sub_mock).data = data_mock

        data_mock = PropertyMock(side_effect=[RobotState(),
                                              RobotState()])
        type(attach_sub_mock).data = data_mock

        collision_world.remove_all()
        msg = CollisionObject()
        msg.id = 'chair'
        msg.operation = CollisionObject.REMOVE
        pub_mock.publish.assert_called_with(msg)
