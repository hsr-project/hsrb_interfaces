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

"""Utility classes and functions"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
import threading
import time

import rclpy
from rclpy.duration import Duration

from . import exceptions


class CachingSubscriber(object):
    """Subscribe a topic and keep its latest message for given period."""

    def __init__(self, topic, msg_type, node, time_to_live=0.0, default=None,
                 **kwargs):
        """Initialize a instance

        Args:
            topic (str):                ROS topic name
            msg_type (rospy.Message):   ROS message type
            time_to_live (double):      Time to live of a latest message [sec]
            default (msg_type):         Default value for :py:attr:`.data`
            kwargs (Dict[str, object]): Options passed to rospy.Subscriber
        """
        self._lock = threading.Lock()
        self._node = node
        self._time_to_live = Duration(seconds=time_to_live)
        self._latest_stamp = self._node.get_clock().now()
        self._default = default
        self._msg = default
        self._topic = topic
        self._msg_type = msg_type
        self._sub = self._node.create_subscription(
            msg_type, topic, self._callback, 1, **kwargs)

    def wait_for_message(self, timeout=None):
        """Wait for a new meesage until elapsed time exceeds ``timeout`` [sec].

        If ``timeout`` is None, a instance wait infinitely.

        Returns:
            None
        """
        self._msg = self._default
        if timeout is not None:
            timeout_t = time.time() + timeout
            while rclpy.ok() and self._msg == self._default:
                time.sleep(0.01)
                if time.time() >= timeout_t:
                    raise exceptions.RobotConnectionError(
                        "timeout exceeded while waiting for message "
                        "on topic %s" % self._topic)
        else:
            while rclpy.ok() and self._msg == self._default:
                time.sleep(0.01)

    def _callback(self, msg):
        """Subscriber callback"""
        if self._lock.acquire(False):
            try:
                self._msg = msg
                self._latest_stamp = self._node.get_clock().now()
            finally:
                self._lock.release()

    @property
    def data(self):
        """(Message Type): Latest topic value"""
        with self._lock:
            if self._time_to_live.nanoseconds != 0:
                now = self._node.get_clock().now()
                if (now - self._latest_stamp) > self._time_to_live:
                    self._msg = self._default
            return copy.deepcopy(self._msg)


def iterate(func, times=None):
    """Create a generator that yields result of ``func`` calls ``times``-times.

    Args:
        func (callable): a callable object repeatedly invoked
        times (int): Number of calls (Infinte if None)
    Returns:
        A generator object
    """
    if times is None:
        while True:
            yield func()
    else:
        for _ in range(times):
            yield func()
