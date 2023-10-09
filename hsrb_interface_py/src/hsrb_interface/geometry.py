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

"""This module implements  simple geometry utilities.

Notes:
    At present, this module does not provide adequate features for serious
    usage. Please consider using other libraries for such kind of task.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import collections
import math
import warnings

from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import Transform as RosTransform
import numpy as np
import tf

from . import exceptions

Vector3 = collections.namedtuple('Vector3', 'x y z')
Quaternion = collections.namedtuple('Quaternion', 'x y z w')
Pose = collections.namedtuple('Pose', 'pos ori')


def pose(x=0.0, y=0.0, z=0.0, ei=0.0, ej=0.0, ek=0.0, axes='sxyz'):
    """Create a new pose-tuple representation.

    Args:
        x, y, z: Linear translation.
        ei, ej, ek, axes: Rotation in euler form.
            By default, (ei, ej, ek) are correspond to (roll, pitch, yaw).

    Returns:
        Tuple[Vector3, Quaternion]: A new pose.
    """
    vec3 = (x, y, z)
    quaternion = tf.transformations.quaternion_from_euler(ei, ej, ek, axes)
    return Pose(Vector3(*vec3), Quaternion(*quaternion))


def create_pose(x=0.0, y=0.0, z=0.0, ei=0.0, ej=0.0, ek=0.0, axes='sxyz'):
    """Create a new pose-tuple representation.

    Args:
        x, y, z: Linear translation.
        ei, ej, ek, axes: Rotation in euler form.
            By default, (ei, ej, ek) are correspond to (roll, pitch, yaw).

    Warning:
        This function is deprecated. Use :py:func:`pose()` instead.
    """
    warnings.warn('create_pose() is deprecated. Use pose() instead.',
                  exceptions.DeprecationWarning)
    return pose(x, y, z, ei, ej, ek, axes)


def vector3(x=0.0, y=0.0, z=0.0):
    """Construct a Vector3 instance.

    Returns:
        Vector3: A new :py:class:`Vector3` instance.
    """
    return Vector3(x, y, z)


def quaternion(x=0.0, y=0.0, z=0.0, w=1.0):
    """Construct a Quaternion instance.

    Returns:
        Quaternion: A new :py:class:`Quaternion` instance.
    """
    return Quaternion(x, y, z, w)


def from_ros_vector3(msg):
    """Convert ``geometry_msgs/Vector3`` to a :py:class:`Vector3`.

    Args:
        msg (geometry_msgs.msg.Vector3): A ROS message.

    Returns:
        Vector3: A new :py:class:`Vector3` instance.
    """
    return Vector3(msg.x, msg.y, msg.z)


def from_ros_quaternion(msg):
    """Convert ``geometry_msgs/Quaternion`` to a :py:class:`Quaternion`.

    Args:
        msg (geometry_msgs.msg.Quaternion): A ROS message.

    Returns:
        Quaternion: A new :py:class:`Quaternion` instance.
    """
    return Quaternion(msg.x, msg.y, msg.z, msg.w)


def normalize_angle_positive(angle):
    """Normalize a given angle value to 0 to 2PI range.

    Args:
        angle (float): An input angle.

    Returns:
        float: A normalized angle.
    """
    twopi = 2.0 * math.pi
    return math.fmod(math.fmod(angle, twopi) + twopi, twopi)


def normalize_angle(angle):
    """Normalize a given angle value to -PI to PI range.

    Args:
        angle (float): An input angle.

    Returns:
        float: A normalized angle.
    """
    a = normalize_angle_positive(angle)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


def shortest_angular_distance(_from, to):
    """Compute shortest angular distance of 2 angles.

    Args:
        _from (float): A source angle.
        to (float): A target angle.

    Returns:
        float: A distance of 2 angles.
    """
    return normalize_angle(to - _from)


def tuples_to_pose(tuples):
    """Convert a pose-tuple representation to a ``geometry_msgs/Pose``.

    Args:
        tuples (Tuple[Vector3, Quaternion]): A pose-tuple representation.

    Returns:
        geometry_msgs.msg.Pose: A result of conversion.
    """
    trans, rot = tuples
    pose = RosPose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose


def pose_to_tuples(pose):
    """Convert a ``geometry_msgs/Pose`` to a pose-tuple representation.

    Args:
        pose (geometry_msgs.msg.Pose): A pose message.

    Returns:
        tuples (Tuple[Vector3, Quaternion]): A result of conversion.
    """
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    return Pose(Vector3(x, y, z), Quaternion(qx, qy, qz, qw))


def tuples_to_transform(tuples):
    """Convert a pose-tuple representation to a ``geometry_msgs/Transform``.

    Args:
        tuples (Tuple[Vector3, Quaternion]): A pose-tuple representation.

    Returns:
        geometry_msgs.msg.Transform: A result of conversion.
    """
    trans, rot = tuples
    transform = RosTransform()
    transform.translation.x = trans[0]
    transform.translation.y = trans[1]
    transform.translation.z = trans[2]
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]
    transform.rotation.w = rot[3]
    return transform


def transform_to_tuples(transform):
    """Convert a ``geometry_msgs/Transform`` to a pose-tuple representation.

    Args:
        transform (geometry_msgs.msg.Transform): A transform message.

    Returns:
        tuples (Tuple[Vector3, Quaternion]): A result of conversion.
    """
    x = transform.translation.x
    y = transform.translation.y
    z = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    return Pose(Vector3(x, y, z), Quaternion(qx, qy, qz, qw))


def multiply_tuples(t1, t2):
    """Multiply 2 pose-tuple representation.

    Args:
        t1 (Tuple[Vector3, Quaternion]): Left hand side operand.
        t2 (Tuple[Vector3, Quaternion]): Right hand side operand.

    Returns:
        Tuple[Vector3, Quaternion]: A result of multiplication.
    """
    trans1, rot1 = t1
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2, rot2 = t2
    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return Pose(Vector3(*trans3), Quaternion(*rot3))
