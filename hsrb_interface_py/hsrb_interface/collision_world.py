# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
# vim: fileencoding=utf-8
"""Collision checking interface."""

import os
import time
import warnings

from geometry_msgs.msg import Point
from hsrb_interface import geometry

from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningSceneWorld
from moveit_msgs.msg import RobotState
import numpy as np
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

import rclpy

from shape_msgs.msg import (
    Mesh,
    MeshTriangle,
    SolidPrimitive,
)
from std_msgs.msg import String

from . import robot
from . import settings
from . import utils

# To suppress FutureWarning from numpy caused by importing mesh
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', category=FutureWarning, module='importlib._bootstrap')
    from stl import mesh

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
        self._trans_env_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)

        self._attaching_pub = {}
        self._add_attaching_pub = {}
        self._releasing_pub = {}
        self._attach_info_sub = {}

        for frame_id in self._setting["attached_object"].keys():
            self._attaching_pub[frame_id] = self._node.create_publisher(
                String,
                self._setting["attached_object"][frame_id]['attaching_topic'],
                100
            )
            self._add_attaching_pub[frame_id] = self._node.create_publisher(
                AttachedCollisionObject,
                self._setting["attached_object"][frame_id]['add_attaching_topic'],
                100
            )
            self._releasing_pub[frame_id] = self._node.create_publisher(
                String,
                self._setting["attached_object"][frame_id]['releasing_topic'],
                100
            )
            self._attach_info_sub[frame_id] = utils.CachingSubscriber(
                self._setting["attached_object"][frame_id]['attached_info_topic'],
                RobotState,
                default=RobotState()
            )
            self._attach_info_sub[frame_id].wait_for_message(_WAIT_TOPIC_TIMEOUT)

    def _is_object_id_used(self, object_id):
        """Check if a given object ID is used or not"""
        known_ids = [x.id for x in self._environment_sub.data.collision_objects]
        return object_id in known_ids

    def _is_object_id_attached(self, frame_id, object_id):
        """Check if a given object ID is attached or not"""
        known_ids = [x.object.id for x in self._attach_info_sub[frame_id].data.attached_collision_objects]
        return object_id in known_ids

    def _create_collision_object(self, obj, pose, name, frame_id):
        collision_obj = CollisionObject()
        collision_obj.operation = CollisionObject.ADD
        collision_obj.id = name

        objects = []
        poses = []
        if isinstance(pose, list):
            objects = [obj for _ in pose]
            poses = [geometry.tuples_to_pose(pos) for pos in pose]
        else:
            objects = [obj]
            poses = [geometry.tuples_to_pose(pose)]

        if isinstance(obj, Mesh):
            collision_obj.meshes = objects
            collision_obj.mesh_poses = poses
        else:
            collision_obj.primitives = objects
            collision_obj.primitive_poses = poses

        collision_obj.header.frame_id = frame_id
        collision_obj.header.stamp = rclpy.time.Time().to_msg()
        return collision_obj

    def _create_mesh(self, filename):
        if not os.path.exists(filename):
            raise ValueError("stl file does not exist.")

        mesh_data = mesh.Mesh.from_file(filename)
        all_vertices = np.vstack([mesh_data.v0, mesh_data.v1, mesh_data.v2])
        vertices, indices = np.unique(all_vertices, return_inverse=True, axis=0)
        indices = indices.reshape(3, -1).transpose()

        mesh_msg = Mesh()
        for vertex in vertices:
            point = Point()
            point.x, point.y, point.z = vertex.astype(np.float64)
            mesh_msg.vertices.append(point)
        for index in indices:
            triangle = MeshTriangle()
            triangle.vertex_indices = index.tolist()
            mesh_msg.triangles.append(triangle)
        return mesh_msg

    def _wait_object_id_used(self, object_id, timeout=1.0):
        timeout_sec = timeout
        while rclpy.ok() and timeout_sec > 0.0:
            rclpy.spin_once(self._node)
            if self._is_object_id_used(object_id):
                return True

            time.sleep(0.01)
            timeout_sec -= 0.01

        return False

    def _wait_object_id_attached(self, frame_id, object_id, timeout=1.0):
        timeout_sec = timeout
        while rclpy.ok() and timeout_sec > 0.0:
            rclpy.spin_once(self._node)
            if self._is_object_id_attached(frame_id, object_id):
                return True

            time.sleep(0.01)
            timeout_sec -= 0.01

        return False

    def _wait_object_id_released(self, frame_id, object_id, timeout=1.0):
        timeout_sec = timeout
        while rclpy.ok() and timeout_sec > 0.0:
            rclpy.spin_once(self._node)
            if not self._is_object_id_attached(frame_id, object_id):
                return True

            time.sleep(0.01)
            timeout_sec -= 0.01

        return False

    def _wait_object_id_released_all(self, frame_id, timeout=1.0):
        timeout_sec = timeout
        while rclpy.ok() and timeout_sec > 0.0:
            rclpy.spin_once(self._node)
            if len(self._attach_info_sub[frame_id].data.attached_collision_objects) == 0:
                return True

            time.sleep(0.01)
            timeout_sec -= 0.01

        return False

    def _add_object(self, obj, pose, name, frame_id, timeout):
        collision_object = self._create_collision_object(obj, pose, name, frame_id)
        self._object_pub.publish(collision_object)

        # Wait until it is reflected
        if self._wait_object_id_used(collision_object.id, timeout):
            return collision_object.id
        else:
            return None

    def _add_attached_object(self, obj, pose, name, frame_id, timeout):
        attached_object = AttachedCollisionObject()
        attached_object.object = self._create_collision_object(obj, pose, name, frame_id)
        attached_object.link_name = frame_id
        self._add_attaching_pub[frame_id].publish(attached_object)

        # Wait until it is reflected
        if self._wait_object_id_attached(frame_id, attached_object.object.id, timeout):
            return attached_object.object.id
        else:
            return None

    def _check_end_effector_frames(self, frame_id):
        end_effector_frame_list = self._setting["attached_object"].keys()
        if frame_id not in end_effector_frame_list:
            raise ValueError(f'{frame_id} is not included in [{", ".join(end_effector_frame_list)}]')

        return True

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
        object_list = []
        for frame_id in self._setting["attached_object"].keys():
            object_list.extend(self._attach_info_sub[frame_id].data.attached_collision_objects)

        return object_list

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

        # Change the parameters
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

        # Subscribe to transformed_environment
        self._trans_env_sub.wait_for_message()
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
        # Create a CollisionObject
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
        if self._check_end_effector_frames(frame_id):
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
        if self._check_end_effector_frames(frame_id):
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
        # Create a CollisionObject
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
        if self._check_end_effector_frames(frame_id):
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.CYLINDER
            shape.dimensions = [length, radius]

            return self._add_attached_object(shape, pose, name, frame_id, timeout)

    def add_mesh(self, filename, pose=geometry.pose(), frame_id='map', name='mesh', timeout=1.0):
        """Add a mesh object to the collision space.

        Args:
            filename: An URI to a STL file.
                Acceptable scheme is 'file'.

                Example:
                    - file:///home/hoge/hoge.stl'

            pose: A pose/poses of a new object from the frame ``frame_id`` .
            frame_id: A reference frame of a new object.
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            name (str): A name of an added object.

        Raises:
            ValueError: A file does not exist.
        """
        mesh_obj = self._create_mesh(filename)

        return self._add_object(mesh_obj, pose, name, frame_id, timeout)

    def add_attached_mesh(self, filename, pose=geometry.pose(),
                          frame_id='hand_palm_link', name='mesh', timeout=1.0):
        """Add a mesh object to the collision space.

        Args:
            filename: An URI to a STL file.
                Acceptable scheme is 'file'.

                Example:
                    - file:///home/hoge/hoge.stl'

            pose: A pose/poses of a new object from the frame ``frame_id`` .
            frame_id: A reference end effector frame of a new object.
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            name (str): A name of an added object.

        Raises:
            ValueError: A file does not exist.
            ValueError: frame_id is not end effector frame.
        """
        if self._check_end_effector_frames(frame_id):
            mesh_obj = self._create_mesh(filename)

            return self._add_attached_object(mesh_obj, pose, name, frame_id, timeout)

    def attach(self, object_id, frame_id='hand_palm_link', timeout=1.0):
        """Attach a specified object from the existing object.

        Args:
            object_id (string): A known object ID
            frame_id: A reference end effector frame of attaching object.
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            name (str): A name of an attached object.

        Raises:
            ValueError: frame_id is not end effector frame.
            ValueError: object_id does not exist.
        """
        if self._check_end_effector_frames(frame_id):
            if not self._is_object_id_used(object_id):
                raise ValueError("object_id is not used")

            for collision_object in self._environment_sub.data.collision_objects:
                if collision_object.id == object_id:
                    object_info = String()
                    object_info.data = object_id
                    self._attaching_pub[frame_id].publish(object_info)

                    # Wait until it is reflected
                    if self._wait_object_id_attached(frame_id, object_id, timeout):
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
        # Determine the frame ID holding the object
        frame_id = ''
        for gripper_frame_id in self._setting["attached_object"].keys():
            if self._is_object_id_attached(gripper_frame_id, object_id):
                frame_id = gripper_frame_id
                break

        # If there is no frame ID, it means the object is not being held, so mark it as successful
        if not frame_id:
            return True

        object_info = String()
        object_info.data = object_id
        self._releasing_pub[frame_id].publish(object_info)

        # Wait until it is reflected
        return self._wait_object_id_released(frame_id, object_id, timeout)

    def release_all(self, frame_id='', timeout=1.0):
        """Release a specified object from the attached object.

        Args:
            frame_id (string): A reference end effector frame of attaching object.
            timeout (float): Wait attached object list for this value [sec]

        Returns:
            result (bool): Result of release process.

        Raises:
            ValueError: frame_id is not end effector frame.
        """
        end_effector_frames = []
        if not frame_id:
            # If frame_id is empty, release all objects
            end_effector_frames = self._setting["attached_object"].keys()
        else:
            # Release all objects from the hand specified by frame_id
            if self._check_end_effector_frames(frame_id):
                end_effector_frames.append(frame_id)

        for end_effector_frame_id in end_effector_frames:
            attached_object = AttachedCollisionObject()
            attached_object.object = CollisionObject()
            attached_object.object.operation = CollisionObject.REMOVE
            attached_object.link_name = end_effector_frame_id
            self._add_attaching_pub[end_effector_frame_id].publish(attached_object)

            # Wait until it is reflected
            if not self._wait_object_id_released_all(end_effector_frame_id, timeout):
                return False

        return True

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
        self.release_all(frame_id='', timeout=timeout)

        known_ids = [x.id for x in self._environment_sub.data.collision_objects]

        for known_id in known_ids:
            self.remove(known_id, timeout)
