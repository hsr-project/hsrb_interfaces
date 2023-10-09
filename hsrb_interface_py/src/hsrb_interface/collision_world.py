# vim: fileencoding=utf-8
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

"""Collision checking interface."""


from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals


from hsrb_interface import geometry
import rospy
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_manipulation_msgs.msg import CollisionEnvironment
from tmc_manipulation_msgs.msg import CollisionObject
from tmc_manipulation_msgs.msg import CollisionObjectOperation
from tmc_manipulation_msgs.srv import GetCollisionEnvironment
from tmc_manipulation_msgs.srv import GetCollisionEnvironmentRequest
from tmc_msgs.msg import ObjectIdentifierArray

from . import robot
from . import settings
from . import utils

# Timeout to wait for message [sec]
_WAIT_TOPIC_TIMEOUT = 20.0

_UINT32_MAX = 4294967295


class CollisionWorld(robot.Item):
    """Abstract interface that represents collision space.

    The collision space is usually unique and global.

    Attributes:
        known_object_only (bool):
            If True :py:meth:`.snapshot` exclude unknown objects.
        ref_frame_id (str):
            A reference frame ID for a snapshot.
        next_object_id (int):
            Next ID that may be given to a newly added collision object.
    """

    def __init__(self, name):
        """Intialize an instance.

        Args:
            name (str): A name of a target resouce
        """
        super(CollisionWorld, self).__init__()
        self._setting = settings.get_entry('collision_world', name)
        self._known_object_only = True
        self._ref_frame_id = settings.get_frame('map')
        self._environment = CollisionEnvironment()
        self._object_pub = rospy.Publisher(self._setting['control_topic'],
                                           CollisionObject,
                                           queue_size=100)
        # object_id starts 10000 by default.
        self._start_object_id = _UINT32_MAX
        self._object_count = self._start_object_id
        self._known_obj_ids_sub = utils.CachingSubscriber(
            self._setting['listing_topic'],
            ObjectIdentifierArray,
            default=ObjectIdentifierArray()
        )
        self._known_obj_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)

    def _is_object_id_used(self, object_id):
        """Check if a given object ID is used or not"""
        known_ids = [x.object_id for x in
                     self._known_obj_ids_sub.data.object_ids]
        return object_id in known_ids

    def _wait_object_id_used(self, id, timeout=1.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._is_object_id_used(id):
                return True
            elif rospy.Time.now() - start > rospy.Duration(timeout):
                return False
            rospy.sleep(0.1)

    def _get_known_object_only(self):
        """Getter for :py:attr:`known_object_only`."""
        return self._known_object_only

    def _set_known_object_only(self, value):
        """Setter for :py:attr:`known_object_only`."""
        self._known_object_only = value

    known_object_only = property(_get_known_object_only,
                                 _set_known_object_only)

    def _get_ref_frame_id(self):
        return self._ref_frame_id

    def _set_ref_frame_id(self, value):
        self._ref_frame_id = value

    ref_frame_id = property(_get_ref_frame_id, _set_ref_frame_id)

    def _get_next_object_id(self):
        return self._object_count

    def _set_next_object_id(self, value):
        self._start_object_id = value
        self._object_count = self._start_object_id

    next_object_id = property(_get_next_object_id, _set_next_object_id)

    @property
    def environment(self):
        """CollisionEnvironment: A latest snapshot of a collision world."""
        return self._environment

    def snapshot(self, ref_frame_id=None):
        """Get a snapshot of collision space from present environment.

        Args:
            ref_frame_id (str):　A base frame of a snapshot space.
                This parameter overrides ref_frame_id attribute.

        Returns:
            tmc_manipulation_msgs.msg.CollisionEnvironment:
                A snapshot of collision space.
        """
        req = GetCollisionEnvironmentRequest()
        req.known_object_only = self._known_object_only
        if ref_frame_id is None:
            req.origin_frame_id = self._ref_frame_id
        else:
            req.origin_frame_id = ref_frame_id

        service = rospy.ServiceProxy(self._setting['service'],
                                     GetCollisionEnvironment)
        res = service.call(req)
        self._environment = res.environment
        return self._environment

    def add_box(self, x=0.1, y=0.1, z=0.1, pose=geometry.pose(),
                frame_id='map', name='box', timeout=1.0):
        """Add a box object to the collision space.

        Args:
            x (float): Length along with X-axis [m]
            y (float): Length along with Y-axis [m]
            z (float): Length along with Z-axis [m]
            pose (Tuple[Vector3, Quaternion]):
                A pose of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            Tuple[int, str]: ID and name of an added object.
        """
        box = CollisionObject()
        shape = Shape()
        shape.type = Shape.BOX
        shape.dimensions = [x, y, z]
        pose = geometry.tuples_to_pose(pose)
        box.operation.operation = CollisionObjectOperation.ADD
        self._known_obj_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count - 1
        box.id.object_id = self._object_count
        box.id.name = name
        box.shapes = [shape]
        box.poses = [pose]
        box.header.frame_id = frame_id
        box.header.stamp = rospy.Time.now()
        self._object_pub.publish(box)
        # Wait for reflection
        if self._wait_object_id_used(box.id.object_id, timeout):
            return (box.id.object_id, box.id.name)
        else:
            return None

    def add_sphere(self, radius=0.1, pose=geometry.pose(),
                   frame_id='map', name='sphere', timeout=1.0):
        """Add a sphere object to the collision space.

        Args:
            radius: Radius [m]
            pose (Tuple[Vector3, Quaternion]):
                A pose of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            Tuple[int, str]: ID and name of an added object.
        """
        sphere = CollisionObject()
        shape = Shape()
        shape.type = Shape.SPHERE
        shape.dimensions = [radius]
        pose = geometry.tuples_to_pose(pose)
        sphere.operation.operation = CollisionObjectOperation.ADD
        self._known_obj_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count - 1
        sphere.id.object_id = self._object_count
        sphere.id.name = name
        sphere.shapes = [shape]
        sphere.poses = [pose]
        sphere.header.frame_id = frame_id
        sphere.header.stamp = rospy.Time.now()
        self._object_pub.publish(sphere)
        # Wait for reflection
        if self._wait_object_id_used(self._object_count, timeout):
            return (sphere.id.object_id, sphere.id.name)
        else:
            return None

    def add_cylinder(self, radius=0.1, length=0.1, pose=geometry.pose(),
                     frame_id='map', name='cylinder', timeout=1.0):
        """Add a cylinder object to the collision space.

        Args:
            radius: Radius [m]
            length: Height [m]
            pose (Tuple[Vector3, Quaternion]):
                A pose of a new object from the frame ``frame_id``
            frame_id (str): A reference frame of a new object
            name (str): A name of a new object
            timeout (float): Wait known object list for this value [sec]

        Returns:
            Tuple[int, str]: ID and name of an added object.
        """
        cylinder = CollisionObject()
        shape = Shape()
        shape.type = Shape.CYLINDER
        shape.dimensions = [radius, length]
        pose = geometry.tuples_to_pose(pose)
        cylinder.operation.operation = CollisionObjectOperation.ADD
        self._known_obj_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count - 1
        cylinder.id.object_id = self._object_count
        cylinder.id.name = name
        cylinder.shapes = [shape]
        cylinder.poses = [pose]
        cylinder.header.frame_id = frame_id
        cylinder.header.stamp = rospy.Time.now()
        self._object_pub.publish(cylinder)
        # Wait for reflection
        if self._wait_object_id_used(self._object_count, timeout):
            return (cylinder.id.object_id, cylinder.id.name)
        else:
            return None

    def add_mesh(self, filename, pose=geometry.pose(), frame_id='map',
                 name='mesh', timeout=1.0):
        """Add a mesh object to the collision space.

        Args:
            filename: An URI to a STL file.
                Acceptable schemes are 'http', 'package', 'file'.

                Example:

                    - http://hoge/mesh.stl
                    - package://your_pkg/mesh/hoge.stl
                    - file:///home/hoge/huge.stl'

            pose: A pose of a new object from the frame ``frame_id`` .
            frame_id: A reference frame of a new object.

        Returns:
            Tuple[int, str]: ID and name of an added object.

        Raises:
            IOError: A file does not exist.
        """
        mesh = CollisionObject()
        shape = Shape()
        shape.type = Shape.MESH
        shape.stl_file_name = filename
        pose = geometry.tuples_to_pose(pose)
        mesh.operation.operation = CollisionObjectOperation.ADD
        self._known_obj_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count - 1
        mesh.id.object_id = self._object_count
        mesh.id.name = name
        mesh.shapes = [shape]
        mesh.poses = [pose]
        mesh.header.frame_id = frame_id
        mesh.header.stamp = rospy.Time.now()
        self._object_pub.publish(mesh)
        # Wait for reflection
        if self._wait_object_id_used(self._object_count, timeout):
            return (mesh.id.object_id, mesh.id.name)
        else:
            return None

    def remove(self, object_id):
        """Remove a specified object from the collision space.

        Args:
            object_id (Tuple[int, str]): A known object ID

        Returns:
            None
        """
        number, name = object_id
        collision_object = CollisionObject()
        collision_object.id.object_id = number
        collision_object.id.name = name
        collision_object.operation.operation = CollisionObjectOperation.REMOVE
        self._object_pub.publish(collision_object)

    def remove_all(self):
        """Remove all collision objects

        Returns:
            None
        """
        clear = CollisionObject()
        clear.header.stamp = rospy.Time.now()
        clear.id.object_id = 0
        clear.id.name = 'all'
        clear.operation.operation = CollisionObjectOperation.REMOVE
        self._object_count = self._start_object_id
        self._object_pub.publish(clear)
