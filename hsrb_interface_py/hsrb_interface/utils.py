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
"""Utility classes and functions"""

import asyncio
import copy
import threading
import time

import action_msgs.msg as action_msgs
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from . import robot


class CachingSubscriber(robot.Item):
    """Subscribe a topic and keep its latest message for given period."""

    def __init__(self, topic, msg_type, time_to_live=0.0, default=None,
                 qos_profile=QoSProfile(
                     reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     depth=1)):
        """Initialize a instance

        Args:
            topic (str):                ROS topic name
            msg_type (rclpy.Message):   ROS message type
            time_to_live (double):      Time to live of a latest message [sec]
            default (msg_type):         Default value for :py:attr:`.data`
            kwargs (Dict[str, object]): Options passed to rclpy.Subscriber
        """
        super(CachingSubscriber, self).__init__()
        self._lock = threading.Lock()
        self._time_to_live = rclpy.duration.Duration(seconds=time_to_live)
        self._latest_stamp = self._node.get_clock().now().to_msg()
        self._default = default
        self._msg = default
        self._topic = topic
        self._msg_type = msg_type
        self.message_count = 0
        self._node.create_subscription(msg_type, topic, self._callback, qos_profile)

    def wait_for_message(self, timeout=None, sleep_time=0.01):
        """Wait for a new meesage until elapsed time exceeds ``timeout`` [sec].

        If ``timeout`` is None, a instance wait infinitely.

        Args:
            timeout (double):    Max wait time [sec]
            sleep_time (double): Time for sleep [sec]

        Returns:
            None
        """
        timeout_sec = timeout
        if timeout_sec is None:
            timeout_sec = float('inf')
        self.message_count = 0
        while self._node.context.ok() and not self.message_count != 0 and timeout_sec > 0.0:
            rclpy.spin_once(self._node)
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

    def _callback(self, msg):
        self.message_count += 1
        if self._lock.acquire(False):
            try:
                self._msg = msg
                self._latest_stamp = self._node.get_clock().now().to_msg()
            finally:
                self._lock.release()

    @property
    def data(self):
        """(Message Type): Latest topic value"""
        with self._lock:
            is_zero = self._time_to_live.nanoseconds == 0
            if not is_zero:
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


def get_parameters_from_another_node(
        node, param_service_name, parameter_names):
    client = node.create_client(GetParameters, param_service_name)
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')
    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))

    return_values = []
    for pvalue in response.values:
        if pvalue.type == ParameterType.PARAMETER_BOOL:
            value = pvalue.bool_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER:
            value = pvalue.integer_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
            value = pvalue.double_value
        elif pvalue.type == ParameterType.PARAMETER_STRING:
            value = pvalue.string_value
        elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = pvalue.byte_array_value
        elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = pvalue.bool_array_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = pvalue.integer_array_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = pvalue.double_array_value
        elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
            value = pvalue.string_array_value
        elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            raise RuntimeError("Unknown parameter type '{pvalue.type}'"
                               .format_map(locals()))
        return_values.append(value)

    return return_values


def wait_until_complete(node, future, timeout=None):
    timeout_sec = timeout
    sleep_time = 0.25
    if timeout_sec is None:
        timeout_sec = float('inf')
    res = None
    while node.context.ok() and timeout_sec > 0.0:
        rclpy.spin_until_future_complete(node, future, timeout_sec=sleep_time)
        res = future.result()
        if res is not None:
            break
        timeout_sec -= sleep_time
    return res


def get_transform(node, tf2_buffer, target_frame, source_frame, timeout=None):
    # Wait for the current time tf
    tf_future = tf2_buffer.wait_for_transform_async(
        target_frame=target_frame,
        source_frame=source_frame,
        time=node.get_clock().now()
    )

    wait_until_complete(node, tf_future, timeout)

    # Retrieve the latest available tf
    transform = asyncio.run(tf2_buffer.lookup_transform_async(
        target_frame=target_frame,
        source_frame=source_frame,
        time=rclpy.time.Time()
    ))

    return transform


def get_action_state(node, future, timeout=0.1):
    """Get a status of the action client"""
    goal_handle = future.result()
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=timeout)
    res = get_result_future.result()
    if res is None:
        return action_msgs.GoalStatus.STATUS_EXECUTING
    else:
        return res.status


def get_action_state_text(status):
    status_strings = {
        action_msgs.GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",  # noqa
        action_msgs.GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",  # noqa
        action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
        action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
        action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
        action_msgs.GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",  # noqa
        action_msgs.GoalStatus.STATUS_ABORTED: "STATUS_ABORTED"  # noqa
    }
    return status_strings[status]
