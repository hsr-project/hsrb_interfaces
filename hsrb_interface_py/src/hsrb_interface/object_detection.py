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

"""Provides object detection elements.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
import threading

import rospy
from tmc_vision_msgs.msg import RecognizedObject

from . import geometry
from . import robot
from . import settings

_TF_TIMEOUT = 5.0


def _expired(now, expiration, stamped):
    """Check a given object is outdated or not.

    Args:
        now (rospy.Time):
            A current time stamp
        expiration (rospy.Duration):
            The time to live
        stamped (Stamped Message Type):
            A ROS message type that has a header(stamped)
    Returns:
        bool: True if the object is outdated.
    """
    return (now - stamped.header.stamp) > expiration


class Object(object):
    """Abstract representation of a recognized object.

    Attributes:
        id (int): A ID of this object
    """

    def __init__(self, data):
        """Initialize from a ROS message

        Args:
            data (tmc_vision_msgs.msg.RecognizedObject):
                A ROS message of a recognized object
        """
        self._data = data

    def to_ros(self):
        """Convert ot ROS message type

        Returns:
            tmc_vision_msgs.msg.RecognizedObject: A ROS message reresentation.
        """
        return copy.deepcopy(self._data)

    def __repr__(self):
        """Human friendly object representation."""
        return "<{0}: id={1} name={2}>".format(self.__class__.__name__,
                                               self._data.object_id.object_id,
                                               self._data.object_id.name)

    def __eq__(self, other):
        """Equality is checked by only ``object_id`` ."""
        return self._data.object_id == other._data.object_id

    def get_pose(self, ref_frame_id=None):
        """Get a pose of this object based onf `ref_frame_id`.

        Args:
            ref_frame_id (str): A reference frame of a pose to be returned.

        Returns:
            Tuple[Vector3, Quaternion]: A pose of this object.
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')
        msg = self._data

        camera_to_marker_pose = msg.object_frame
        tf2_buffer = robot._get_tf2_buffer()
        ref_to_camera_tf = tf2_buffer.lookup_transform(
            ref_frame_id,
            msg.header.frame_id,
            msg.header.stamp,
            rospy.Duration(_TF_TIMEOUT))
        return geometry.multiply_tuples(
            geometry.transform_to_tuples(ref_to_camera_tf.transform),
            geometry.pose_to_tuples(camera_to_marker_pose))

    def _get_id(self):
        return self._data.object_id.object_id
    id = property(_get_id)


class ObjectDetector(robot.Item):
    """This object collect and hold results of object detection.

    Attributes:
        expiration (float): Duration to hold newly detected objects [sec].

    Examples:
        .. sourcecode:: python

           with Robot() as robot:
               detector = robot.get("marker", Items.OBJECT_DETECTION)
               objects = detector.get_objects()

    """

    def __init__(self, name):
        """Initialize with a resource of a given `name`.

        Notes:
            This class is not intended to be created by user directly.

        Args:
            name (str): A name of a resource.
        """
        super(ObjectDetector, self).__init__()
        self._setting = settings.get_entry('object_detection', name)
        self._lock = threading.Lock()
        self._cache = {}
        self._expiration = rospy.Duration(10.0)
        self._sub = rospy.Subscriber(self._setting['topic'],
                                     RecognizedObject,
                                     self._callback, queue_size=100)

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._cache[msg.object_id.object_id] = msg
            finally:
                self._lock.release()

    def _get_expiration(self):
        return self._expiration.to_sec()

    def _set_expiration(self, value):
        self._expiration = rospy.Duration(value)

    expiration = property(_get_expiration, _set_expiration,
                          "Duration to hold newly detected objects [sec]")

    def get_objects(self):
        """Get all objects currently detecting.

        Returns:
            Dict[str, Any]:
        """
        with self._lock:
            now = rospy.Time.now()
            new_cache = dict([(k, v) for k, v in self._cache.items()
                              if not _expired(now, self._expiration, v)])
            self._cache = new_cache
            objects = copy.deepcopy(self._cache.values())

        return [Object(o) for o in objects]

    def get_object_by_id(self, id=None):
        """Get a object from current detection pool which has given ID.

        Args:
            id (int): if `id` is  ``None`` .

        Returns:
            A recognized object.
            If ID is not found in the pool, it returns None.
        """
        objects = self.get_objects()
        filtered = [obj for obj in objects if obj.id == id]
        if filtered:
            return filtered[0]
        else:
            return None
