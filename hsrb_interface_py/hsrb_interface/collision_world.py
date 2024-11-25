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
# vim: fileencoding=utf-8
"""Collision checking interface."""

import time

from hsrb_interface import geometry

from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningSceneWorld
from moveit_msgs.msg import RobotState
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

import rclpy

from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from . import robot
from . import settings
from . import utils

# Timeout to wait for message [sec]
_WAIT_TOPIC_TIMEOUT = 3.0


class CollisionWorld(robot.Item):
    """Abstract interface that represents collision space.

    The collision space is usually unique and global.

    Attributes:
        ref_frame_id (str):
            A reference frame ID for a snapshot.
    """

    def __init__(self, name):
        """Intialize an instance.

        Args:
            name (str): A name of a target resouce
        """
        super(CollisionWorld, self).__init__()
        self._setting = settings.get_entry('collision_world', name)
        self._ref_frame_id = settings.get_frame('map')
        self._object_pub = self._node.create_publisher(CollisionObject,
                                                       self._setting['control_topic'],
                                                       100)
        self._environment_sub = utils.CachingSubscriber(
            self._setting['environment_topic'],
            PlanningSceneWorld,
            default=PlanningSceneWorld()
        )
        self._environment_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        self._trans_env_sub = utils.CachingSubscriber(
            self._setting['trans_env_topic'],
            PlanningSceneWorld,
            default=PlanningSceneWorld()
        )
        self._attaching_pub = self._node.create_publisher(String,
                                                          self._setting['attaching_topic'],
                                                          100)
        self._add_attaching_pub = self._node.create_publisher(AttachedCollisionObject,
                                                              self._setting['add_attaching_topic'],
                                                              100)
        self._releasing_pub = self._node.create_publisher(String,
                                                          self._setting['releasing_topic'],
                                                          100)
        self._attach_info_sub = utils.CachingSubscriber(
            self._setting['attached_info_topic'],
            RobotState,
            default=RobotState()
        )
        self._trans_env_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)

    def _is_object_id_used(self, object_id):
        """Check if a given object ID is used or not"""
        known_ids = [x.id for x in self._environment_sub.data.collision_objects]
        return object_id in known_ids

    def _is_object_id_attached(self, object_id):
        """Check if a given object ID is attached or not"""
        known_ids = [x.object.id for x in self._attach_info_sub.data.attached_collision_objects]
        return object_id in known_ids

    def _create_collision_object(self, shape, pose, name, frame_id):
        collision_obj = CollisionObject()
        collision_obj.operation = CollisionObject.ADD
        collision_obj.id = name
        if isinstance(pose, list):
            collision_obj.primitives = [shape for _ in pose]
            collision_obj.primitive_poses = [geometry.tuples_to_pose(pos) for pos in pose]
        else:
            collision_obj.primitives = [shape]
            collision_obj.primitive_poses = [geometry.tuples_to_pose(pose)]
        collision_obj.header.frame_id = frame_id
        return collision_obj

    def _wait_object_id_used(self, id, timeout=1.0):
        start = self._node.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if self._is_object_id_used(id):
                return True
            elif (self._node.get_clock().now() - start) > rclpy.duration.Duration(seconds=timeout):
                return False
            time.sleep(0.1)

    def _wait_object_id_attached(self, id, timeout=1.0):
        start = self._node.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if self._is_object_id_attached(id):
                return True
            elif (self._node.get_clock().now() - start) > rclpy.duration.Duration(seconds=timeout):
                return False
            time.sleep(0.1)

    def _wait_object_id_released(self, id, timeout=1.0):
        start = self._node.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if not self._is_object_id_attached(id):
                return True
            elif (self._node.get_clock().now() - start) > rclpy.duration.Duration(seconds=timeout):
                return False
            time.sleep(0.1)

    def _wait_object_id_released_all(self, timeout=1.0):
        start = self._node.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if len(self._attach_info_sub.data.attached_collision_objects) == 0:
                return True
            elif (self._node.get_clock().now() - start) > rclpy.duration.Duration(seconds=timeout):
                return False
            time.sleep(0.1)

    def _add_object(self, shape, pose, name, frame_id, timeout):
        object = self._create_collision_object(shape, pose, name, frame_id)
        self._object_pub.publish(object)

        # Wait until it is reflected
        if self._wait_object_id_used(object.id, timeout):
            return object.id
        else:
            return None

    def _add_attached_object(self, shape, pose, name, frame_id, timeout):
        attached_object = AttachedCollisionObject()
        attached_object.object = self._create_collision_object(shape, pose, name, frame_id)
        attached_object.link_name = frame_id
        self._add_attaching_pub.publish(attached_object)

        # Wait until it is reflected
        if self._wait_object_id_attached(attached_object.object.id, timeout):
            return attached_object.object.id
        else:
            return None

    def _get_ref_frame_id(self):
        return self._ref_frame_id

    def _set_ref_frame_id(self, value):
        self._ref_frame_id = value

    ref_frame_id = property(_get_ref_frame_id, _set_ref_frame_id)

    @property
    def environment(self):
        """CollisionEnvironment: A latest snapshot of a collision world."""
        return self._environment_sub.data

    @property
    def attached_objects(self):
        """List (AttachedCollisionObject): A latest List of a attaced objects."""
        return self._attach_info_sub.data.attached_collision_objects

    def snapshot(self, ref_frame_id=None):
        """Get a snapshot of collision space from present environment.

        Args:
            ref_frame_id (str): A base frame of a snapshot space.
                This parameter overrides ref_frame_id attribute.

        Returns:
            tmc_manipulation_msgs.msg.CollisionEnvironment:
                A snapshot of collision space.
        """
        if ref_frame_id is None:
            origin_frame_id = self._ref_frame_id
        else:
            origin_frame_id = ref_frame_id

        # Change parameters
        client = self._node.create_client(SetParameters,
                                          self._setting['set_frame_service'])

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'origin_frame_id'
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = origin_frame_id
        req.parameters.append(param)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if not res.results[0].successful:
            raise RuntimeError("Cannot set frame_id")

        # Subscribe to Transformed_Environment
        self._trans_env_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        return self._trans_env_sub.data

    def add_box(self, x=0.1, y=0.1, z=0.1, pose=geometry.pose(),
                frame_id='map', name='box', timeout=1.0):
        """Add a box object to the collision space.

        Args:
            x (float): Length along with X-axis [m]
            y (float): Length along with Y-axis [m]
            z (float): Length along with Z-axis [m]
            pose (Tuple[Vector3, Quaternion] or List of Tuple[Vector3, Quaternion]):
                A pose/poses of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            Tuple[int, str]: ID and name of an added object.
        """
        # Create CollisionObject
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [x, y, z]

        return self._add_object(shape, pose, name, frame_id, timeout)

    def add_attached_box(self, x=0.1, y=0.1, z=0.1, pose=geometry.pose(),
                         frame_id='hand_palm_link', name='box', timeout=1.0):
        """Add a box object to the collision space.

        Args:
            x (float): Length along with X-axis [m]
            y (float): Length along with Y-axis [m]
            z (float): Length along with Z-axis [m]
            pose (Tuple[Vector3, Quaternion] or List of Tuple[Vector3, Quaternion]):
                A pose/poses of a new object from the frame ``frame_id``
            frame_id (str): A reference end effector frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            name (str): A name of an added object.

        Raises:
            ValueError: frame_id is not end effector frame.
        """
        if frame_id not in settings.get_entry('joint_group', 'whole_body')['end_effector_frames']:
            raise ValueError("frame_id is not end effector frame.")

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [x, y, z]

        return self._add_attached_object(shape, pose, name, frame_id, timeout)

    def add_sphere(self, radius=0.1, pose=geometry.pose(),
                   frame_id='map', name='sphere', timeout=1.0):
        """Add a sphere object to the collision space.

        Args:
            radius: Radius [m]
            pose (Tuple[Vector3, Quaternion] or List of Tuple[Vector3, Quaternion]):
                A pose/poses of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            Tuple[int, str]: ID and name of an added object.
        """
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [radius]

        return self._add_object(shape, pose, name, frame_id, timeout)

    def add_attached_sphere(self, radius=0.1, pose=geometry.pose(),
                            frame_id='hand_palm_link', name='sphere', timeout=1.0):
        """Add a sphere object to the collision space.

        Args:
            radius: Radius [m]
            pose (Tuple[Vector3, Quaternion] or List of Tuple[Vector3, Quaternion]):
                A pose/poses of a new object from the frame ``frame_id``
            frame_id (str): A reference end effector frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            name (str): A name of an added object.

        Raises:
            ValueError: frame_id is not end effector frame.
        """
        if frame_id not in settings.get_entry('joint_group', 'whole_body')['end_effector_frames']:
            raise ValueError("frame_id is not end effector frame.")

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.SPHERE
        shape.dimensions = [radius]

        return self._add_attached_object(shape, pose, name, frame_id, timeout)

    def add_cylinder(self, radius=0.1, length=0.1, pose=geometry.pose(),
                     frame_id='map', name='cylinder', timeout=1.0):
        """Add a cylinder object to the collision space.

        Args:
            radius: Radius [m]
            length: Height [m]
            pose (Tuple[Vector3, Quaternion] or List of Tuple[Vector3, Quaternion]):
                A pose/poses of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            Tuple[int, str]: ID and name of an added object.
        """
        # Create CollisionObject
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [length, radius]

        return self._add_object(shape, pose, name, frame_id, timeout)

    def add_attached_cylinder(self, radius=0.1, length=0.1, pose=geometry.pose(),
                              frame_id='hand_palm_link', name='cylinder', timeout=1.0):
        """Add a cylinder object to the collision space.

        Args:
            radius: Radius [m]
            length: Height [m]
            pose (Tuple[Vector3, Quaternion] or List of Tuple[Vector3, Quaternion]):
                A pose/poses of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            name (str): A name of an added object.

        Raises:
            ValueError: frame_id is not end effector frame.
        """
        if frame_id not in settings.get_entry('joint_group', 'whole_body')['end_effector_frames']:
            raise ValueError("frame_id is not end effector frame.")

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [length, radius]

        return self._add_attached_object(shape, pose, name, frame_id, timeout)

    # def add_mesh(self, filename, pose=geometry.pose(), frame_id='map',
    #              name='mesh', timeout=1.0):
    #     """Add a mesh object to the collision space.

    #     Args:
    #         filename: An URI to a STL file.
    #             Acceptable schemes are 'http', 'package', 'file'.

    #             Example:

    #                 - http://hoge/mesh.stl
    #                 - package://your_pkg/mesh/hoge.stl
    #                 - file:///home/hoge/huge.stl'

    #         pose: A pose/poses of a new object from the frame ``frame_id`` .
    #         frame_id: A reference frame of a new object.

    #     Returns:
    #         Tuple[int, str]: ID and name of an added object.

    #     Raises:
    #         IOError: A file does not exist.
    #     """
    #     self._known_obj_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
    #     while self._is_object_id_used(self._object_count):
    #         self._object_count = self._object_count - 1
    #     # Create CollisionObject
    #     shape = Shape()
    #     shape.type = Shape.MESH
    #     shape.stl_file_name = filename
    #     mesh = self._create_collision_object(shape, pose, name, frame_id)
    #     self._object_pub.publish(mesh)
    #     # Wait until it is reflected
    #     if self._wait_object_id_used(self._object_count, timeout):
    #         return (mesh.id.object_id, mesh.id.name)
    #     else:
    #         return None

    def attach(self, object_id, timeout=1.0):
        """Attach a specified object from the existing object.

        Args:
            object_id (string): A known object ID
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            name (str): A name of an attached object.

        Raises:
            ValueError: object_id does not exist.
        """
        if not self._is_object_id_used(object_id):
            raise ValueError("object_id is not used")

        for collision_object in self._environment_sub.data.collision_objects:
            if collision_object.id == object_id:
                object_info = String()
                object_info.data = object_id
                self._attaching_pub.publish(object_info)

                # Wait until it is reflected
                if self._wait_object_id_attached(object_id, timeout):
                    return object_id
                else:
                    return None

    def release(self, object_id, timeout=1.0):
        """Release a specified object from the attached object.

        Args:
            object_id (string): A known object ID
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            result (bool): Result of release process.
        """
        object_info = String()
        object_info.data = object_id
        self._releasing_pub.publish(object_info)

        # Wait until it is reflected
        return self._wait_object_id_released(object_id, timeout)

    def release_all(self, timeout=1.0):
        """Release a specified object from the attached object.

        Args:
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            result (bool): Result of release process.
        """
        attached_object = AttachedCollisionObject()
        attached_object.object = CollisionObject()
        attached_object.object.operation = CollisionObject.REMOVE
        attached_object.link_name = 'hand_palm_link'
        self._add_attaching_pub.publish(attached_object)

        # Wait until it is reflected
        return self._wait_object_id_released_all(timeout)

    def remove(self, object_id, timeout=1.0):
        """Remove a specified object from the collision space.

        Args:
            object_id (string): A known object ID
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            None
        """
        self.release(object_id, timeout)
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        self._object_pub.publish(collision_object)

    def remove_all(self, timeout=1.0):
        """Remove all collision objects

        Args:
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            None
        """
        self.release_all(timeout)

        known_ids = [x.id for x in self._environment_sub.data.collision_objects]

        for known_id in known_ids:
            self.remove(known_id)
