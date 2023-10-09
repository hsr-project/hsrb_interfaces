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

"""This module provides classes and functions to manage connections to robots.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import enum
import importlib
import sys
import warnings
import weakref

import rospy
import tf2_ros

from . import exceptions
from . import settings


class Item(object):
    """A base class to be under resource management.

    Raises:
        hsrb_interface.exceptions.RobotConnectionError:
            Not connected to a robot.
    """

    def __init__(self):
        """See class docstring."""
        if not Robot._connecting():
            raise exceptions.RobotConnectionError("No robot connection")


class ItemTypes(enum.Enum):
    """Types of resource items.

    Attributes:
        JOINT_GROUP:
        END_EFFECTOR:
        MOBILE_BASE:
        CAMERA:
        FORCE_TORQUE:
        IMU:
        LIDAR:
        BATTERY:
        OBJECT_DETECTION:
        COLLISION_WORLD:
        TEXT_TO_SPEECH:

    Warnings:
        This class is deprecated.
    """

    JOINT_GROUP = 'joint_group'
    MOBILE_BASE = 'mobile_base'
    END_EFFECTOR = 'end_effector'
    CAMERA = 'camera'
    FORCE_TORQUE = 'force_torque'
    IMU = 'imu'
    LIDAR = 'lidar'
    BATTERY = 'power_supply'
    OBJECT_DETECTION = 'object_detection'
    COLLISION_WORLD = 'collision_world'
    TEXT_TO_SPEECH = 'text_to_speech'


def _type_deprecation_warning(name, typ):
    """Warn uses of ItemType feature."""
    if typ is not None:
        msg = "A feature specifying an item by ItemType is deprecated."
        warnings.warn(msg, exceptions.DeprecationWarning)
    if name == "default":
        if typ is ItemTypes.TEXT_TO_SPEECH:
            return "default_tts"
        elif typ is ItemTypes.COLLISION_WORLD:
            return "global_collision_world"
    return name


_interactive = False


def _is_interactive():
    """True if interactive mode is set.

    Returns:
        bool: Is interactive mode enabled or not.
    """
    return _interactive


def enable_interactive():
    """Enable interactive mode.

    This function changes behavior when a user send SIGINT(Ctrl-C).
    In the interactive mode, SIGINT doesn't stop process but cancel
    executing action or other blocking procedure.

    Returns:
        None
    """
    global _interactive
    _interactive = True


class _ConnectionManager(object):
    """This class manage connection with a robot.

    Basically, only 1 instance should be created at 1 process.
    Usually ``Robot`` instances manage this object.
    All of ``Resource`` subclass instances are owned by this class and
    synchronize their lifecycle.

    Args:
        use_tf_client: Use action based query for tf.(bool)

    """

    def __init__(self, use_tf_client=False):
        """See class docstring."""
        try:
            master = rospy.get_master()
            master.getUri()  # Examine a master connection
        except Exception as e:
            raise exceptions.RobotConnectionError(e)
        disable_signals = _is_interactive()
        if not rospy.core.is_initialized():
            rospy.init_node('hsrb_interface_py', anonymous=True,
                            disable_signals=disable_signals)
        if use_tf_client:
            self._tf2_buffer = tf2_ros.BufferClient('/tf2_buffer_server')
        else:
            self._tf2_buffer = tf2_ros.Buffer()
            self._tf2_listener = tf2_ros.TransformListener(
                self._tf2_buffer, queue_size=1)
        self._registry = {}

    def __del__(self):
        self._tf2_listener = None
        self._tf2_buffer = None
        rospy.signal_shutdown('shutdown')

    @property
    def tf2_buffer(self):
        return weakref.proxy(self._tf2_buffer)

    def list(self, typ=None):
        """List available items.

        Args:
            typ (ItemTypes):
        """
        if typ is None:
            targets = [x for x in ItemTypes]
        else:
            targets = [typ]
        results = []
        for target in targets:
            section = settings.get_section(target.value)
            if section is None:
                msg = "No such category ({0})".format(target)
                raise exceptions.ResourceNotFoundError(msg)
            for key in section.keys():
                results.append((key, target))
        return results

    def get(self, name, typ=None):
        """Get an item if available.

        Args:
            name (str):   A name of ``Item`` to get.
            typ (Types):  A type of ``Item`` to get.

        Returns:
            Item: An instance with a specified name

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError
        """
        if typ is None:
            section, config = settings.get_entry_by_name(name)
            types = list(filter(lambda e: e.value == section, ItemTypes))
            if types:
                typ = types[0]
            else:
                msg = "No such category ({0})".format(section)
                raise exceptions.ResourceNotFoundError(msg)
        key = (name, typ)
        if key in self._registry:
            return self._registry.get(key, None)
        else:
            config = settings.get_entry(typ.value, name)
            module_name, class_name = config["class"]
            module = importlib.import_module(".{0}".format(module_name),
                                             "hsrb_interface")
            cls = getattr(module, class_name)
            obj = cls(name)
            self._registry[key] = obj
            return weakref.proxy(obj)


def _get_tf2_buffer():
    """Get global tf2 buffer."""
    return Robot._get_tf2_buffer()


class Robot(object):
    """A handle object to manage a robot connection.

    Args:
        *args: Reserved for future use.
        **kwargs: use_tf_client[bool] Use action based query for tf.

    This class allow multiple instances. In that case, the connection is closed
    if all instances are destroyed or invoke :py:meth:`.close()` method.

    In order to establish a new connection, you need to create a new instance.

    Attributes:
        name (str): A name of a connecting robot.

    Example:
        .. sourcecode:: python

           from hsrb_interface import Robot, ItemTypes
                with Robot() as robot:
                    print(robot.list())
                    whole_body = robot.get("whole_body")
    """

    _connection = None

    @property
    def Items(self):
        """Repesent types of resource items.

        Warnings:
            This class is deprecated. It will be removed in future release.
        """
        msg = "A feature specifying a resource item by ItemType is deprecated."
        warnings.warn(msg, exceptions.DeprecationWarning)
        return ItemTypes

    @classmethod
    def _connecting(cls):
        return cls._connection is not None and cls._connection() is not None

    @classmethod
    def connecting(cls):
        """Check whether the connection to a robot is valid."""
        warnings.warn("Robot.connectiong() is depreacated",
                      exceptions.DeprecationWarning)
        return cls._connecting()

    @classmethod
    def _get_tf2_buffer(cls):
        if cls._connection is not None:
            conn = cls._connection()
            if conn is not None:
                return conn.tf2_buffer
            else:
                return None
        else:
            return None

    def __init__(self, *args, **kwargs):
        """See class docstring."""
        use_tf_client = kwargs.get('use_tf_client', False)
        if Robot._connection is None or Robot._connection() is None:
            self._conn = _ConnectionManager(use_tf_client=use_tf_client)
            Robot._connection = weakref.ref(self._conn)
        else:
            self._conn = Robot._connection()

    def close(self):
        """Shutdown immediately."""
        self.__exit__(None, None, None)

    def __enter__(self):
        """A part of ContextManager interface."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """A part of ContextManager interface."""
        self._conn = None

    def ok(self):
        """Check whether this handle is valid or not.

        Returns:
            bool: ``True`` if this handle is valid. Otherwise False.
        """
        return self._conn is not None

    def _get_name(self):
        return settings.get_entry('robot', 'hsrb')['fullname']
    name = property(_get_name)

    def list(self, typ=None):
        """List available items up.

        Args:
            typ (Types):  A type of ``Item`` to list.
                If None, all types are selected.

        Returns:
            List[str]: A list of available items.

        Warnings:
            `typ` parameter is deprecated.
            It will be removed in future release.
        """
        if typ is not None:
            msg = """A feature specifying an item by ItemType is deprecated."""
            warnings.warn(msg, exceptions.DeprecationWarning)
        return self._conn.list(typ)

    def get(self, name, typ=None):
        """Get an item if available.

        Args:
            name (str):   A name of an item to get.
            typ (Types):  A type of an item to get.

        Returns:
            Item: An instance with a specified name.

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError:
                A resource that named as `name` is not found.

        Warnings:
            `typ` parameter is deprecated.
            It will be removed in future release.
        """
        name = _type_deprecation_warning(name, typ)
        return self._conn.get(name)

    def try_get(self, name, typ=None, msg="Ignored"):
        """Try to get an item if available.

        If trial failed, error messsage is printed to ``stderr`` instead of
        raising exception.

        Args:
            name (str):   A name of ``Item`` to get.
            typ (Types):  A type of ``Item`` to get.
            msg (str):  A error message. (If msg is None, output nothing）

        Returns:
            Item: An instance with a specified name.

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError:
                A resource that named as `name` is not found.

        Warnings:
            `typ` parameter is deprecated.
            It will be removed in future release.
        """
        name = _type_deprecation_warning(name, typ)
        try:
            return self.get(name, None)
        except (exceptions.ResourceNotFoundError,
                exceptions.RobotConnectionError):
            if msg is not None:
                err = "Failed to get Item({0}): {1}".format(name, msg)
                print(err, file=sys.stderr)
