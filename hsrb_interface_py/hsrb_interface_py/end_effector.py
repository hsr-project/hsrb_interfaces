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

"""This module contains classes to control end-effector."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import math
import warnings

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from tmc_control_msgs.action import GripperApplyEffort
from trajectory_msgs.msg import JointTrajectoryPoint

from . import exceptions
from . import robot
from . import settings
from . import utils

_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT = 20.0
_GRIPPER_GRASP_TIMEOUT = 20.0
_GRIPPER_APPLY_FORCE_TIMEOUT = 10.0
_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_HAND_MOMENT_ARM_LENGTH = 0.07
_HAND_MOTOR_JOINT_MAX = 1.2
_HAND_MOTOR_JOINT_MIN = -0.5
_JOINT_STATE_SUB_TIMEOUT = 10.0

_DISTANCE_CONTROL_PGAIN = 0.5
_DISTANCE_CONTROL_IGAIN = 1.0
_DISTANCE_CONTROL_RATE = 10.0
_DISTANCE_CONTROL_TIME_FROM_START = 0.2
_DISTANCE_CONTROL_STALL_THRESHOLD = 0.003
_DISTANCE_CONTROL_STALL_TIMEOUT = 1.0

_PALM_TO_PROXIMAL_Y = 0.0245
_PROXIMAL_TO_DISTAL_Z = 0.07
_DISTAL_JOINT_ANGLE_OFFSET = 0.087
_DISTAL_TO_TIP_Y = 0.01865
_DISTAL_TO_TIP_Z = 0.04289

_DISTANCE_MAX = (_PALM_TO_PROXIMAL_Y
                 - (_DISTAL_TO_TIP_Y
                    * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                    + _DISTAL_TO_TIP_Z
                    * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))
                 + _PROXIMAL_TO_DISTAL_Z
                 * math.sin(_HAND_MOTOR_JOINT_MAX)) * 2
_DISTANCE_MIN = (_PALM_TO_PROXIMAL_Y
                 - (_DISTAL_TO_TIP_Y
                    * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                    + _DISTAL_TO_TIP_Z
                    * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))
                 + _PROXIMAL_TO_DISTAL_Z
                 * math.sin(_HAND_MOTOR_JOINT_MIN)) * 2


class Gripper(robot.Item):
    """This class controls 2-finger gripper."""

    def __init__(self, name):
        """Initialize.

        Args:
            name (str): A name of a suction device file.
        """
        super(Gripper, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
        self._isdone = False
        self._state = True
        self._goal_handle = None
        if self._setting is None:
            msg = '{0}({1}) is not found '.format('end_effector', name)
            raise exceptions.ResourceNotFoundError(msg)
        self._name = name
        self._joint_names = self._setting['joint_names']
        prefix = self._setting['prefix']
        self._left_finger_joint_name = self._setting[
            'left_finger_joint_name']
        self._right_finger_joint_name = self._setting[
            'right_finger_joint_name']
        self._follow_joint_trajectory_client = rclpy.action.ActionClient(
            self._node,
            FollowJointTrajectory,
            prefix + "/follow_joint_trajectory")
        self._grasp_client = rclpy.action.ActionClient(
            self._node,
            GripperApplyEffort,
            prefix + "/grasp")
        self._apply_force_client = rclpy.action.ActionClient(
            self._node,
            GripperApplyEffort,
            prefix + "/apply_force")
        self._joint_state_sub = utils.CachingSubscriber(
            "/joint_states",
            JointState,
            self._node
        )
        self._joint_state_sub.wait_for_message(
            timeout=_JOINT_STATE_SUB_TIMEOUT)

        # A variable to remember the last asynchronous operation. Initial value is None.
        self._current_client = None

    def command(self, open_angle, motion_time=1.0, sync=True):
        """Command open a gripper

        Args:
            open_angle (float): How much angle to open[rad]
            motion_time (float): Time to execute command[s]
            sync (bool): Not wait the result when this arg is ``False``

        Returns:
            None

        Example:

            .. sourcecode:: python

               robot = hsrb_interface_py.Robot()
               gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
               gripper.command(1.2, 2.0)
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=Duration(seconds=motion_time).to_msg())
        ]

        self._send_goal(self._follow_joint_trajectory_client, goal)
        self._isdone = False

        if not sync:
            return

        timeout = _GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT * S_TO_NS
        start = self._node.get_clock().now()
        try:
            while not self._isdone:
                time_diff = self._node.get_clock().now() - start
                if time_diff.nanoseconds >= timeout:
                    self.cancel_goal()
                    raise exceptions.GripperError("Timed out")
            if not self._check_state(GoalStatus.STATUS_SUCCEEDED):
                msg = "Failed to follow commanded trajectory"
                raise exceptions.GripperError(msg)
        except KeyboardInterrupt:
            self.cancel_goal()

    def get_distance(self):
        """Command get gripper finger tip distance.

        Returns:
            double: Distance between gripper finger tips [m]
        """
        self._joint_state_sub.wait_for_message(
            timeout=_JOINT_STATE_SUB_TIMEOUT)
        joint_state = self._joint_state_sub.data
        hand_motor_pos = joint_state.position[
            joint_state.name.index(self._joint_names[0])]
        hand_left_position = joint_state.position[
            joint_state.name.index(
                self._left_finger_joint_name)] + hand_motor_pos
        hand_right_position = joint_state.position[
            joint_state.name.index(
                self._right_finger_joint_name)] + hand_motor_pos
        return ((math.sin(hand_left_position)
                 + math.sin(hand_right_position))
                * _PROXIMAL_TO_DISTAL_Z
                + 2 * (_PALM_TO_PROXIMAL_Y
                - (_DISTAL_TO_TIP_Y
                   * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                   + _DISTAL_TO_TIP_Z
                   * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))))

    def set_distance(self, distance, control_time=3.0):
        """Command set gripper finger tip distance.

        Args:
            distance (float): Distance between gripper finger tips [m]
        """
        if distance > _DISTANCE_MAX:
            open_angle = _HAND_MOTOR_JOINT_MAX
            self.command(open_angle)
        elif distance < _DISTANCE_MIN:
            open_angle = _HAND_MOTOR_JOINT_MIN
            self.command(open_angle)
        else:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self._joint_names
            goal.trajectory.points = [
                JointTrajectoryPoint(
                    time_from_start=Duration(
                        seconds=1 / _DISTANCE_CONTROL_RATE).to_msg())
            ]

            start_time = self._node.get_clock().now()
            elapsed_time = Duration(seconds=0.0)
            ierror = 0.0
            theta_ref = math.asin(((distance / 2
                                    - (_PALM_TO_PROXIMAL_Y
                                       - (_DISTAL_TO_TIP_Y
                                          * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                                          + _DISTAL_TO_TIP_Z
                                          * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))))
                                  / _PROXIMAL_TO_DISTAL_Z))
            last_movement_time = self._node.get_clock().now()
            while elapsed_time.nanoseconds / S_TO_NS < control_time:
                try:
                    error = distance - self.get_distance()
                    if abs(error) > _DISTANCE_CONTROL_STALL_THRESHOLD:
                        last_movement_time = self._node.get_clock().now()
                    now = self._node.get_clock().now()
                    if ((now - last_movement_time).nanoseconds / S_TO_NS
                            > _DISTANCE_CONTROL_STALL_TIMEOUT):
                        break
                    ierror += error
                    open_angle = (theta_ref
                                  + _DISTANCE_CONTROL_PGAIN * error
                                  + _DISTANCE_CONTROL_IGAIN * ierror)
                    goal.trajectory.points = [
                        JointTrajectoryPoint(
                            positions=[open_angle],
                            time_from_start=Duration(
                                seconds=_DISTANCE_CONTROL_TIME_FROM_START
                            ).to_msg())
                    ]
                    self._send_goal(self._follow_joint_trajectory_client, goal)
                    self._isdone = False
                    elapsed_time = self._node.get_clock().now() - start_time
                except KeyboardInterrupt:
                    self.cancel_goal()
                    return
                while not self._isdone:
                    elapsed_time = self._node.get_clock().now() - start_time
                    if elapsed_time.nanoseconds / S_TO_NS > control_time:
                        break
            self.cancel_goal()

    def grasp(self, effort):
        """Command a gripper to execute grasping move.

        Args:
            effort (float): Force applied to grasping [Nm]
                            The range is -1[Nm] < effort < 0[Nm]

        Returns:
            None

        Warning:
            This function is deprecated. Use :py:func:`apply_force()` instead.
        """
        msg = ' '.join(["gripper.grasp() is depreacated."
                        "Use gripper.apply_force() instead."])
        warnings.warn(msg, exceptions.DeprecationWarning)
        if effort > 0.0:
            raise exceptions.GripperError("effort shold be negative.")
        else:
            self.apply_force(-effort / _HAND_MOMENT_ARM_LENGTH)

    def apply_force(self, effort, delicate=False, sync=True):
        """Command a gripper to execute applying force.

        Args:
            effort (float): Force applied to grasping [N]
                            'effort' should be positive number
            delicate (bool): Force control is on when delicate is ``True``
                             The range force control works well
                             is 0.2 [N] < effort < 0.6 [N]
            sync (bool): Not wait the result when this arg is ``False``

        Returns:
            None
        """
        if effort < 0.0:
            msg = "negative effort is set"
            raise exceptions.GripperError(msg)
        goal = GripperApplyEffort.Goal()
        goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        client = self._grasp_client
        if delicate:
            if effort < _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD:
                goal.effort = effort
                client = self._apply_force_client
            else:
                self._node.get_logger().warn(
                    "Since effort is high, force control become invalid.")

        self._send_goal(client, goal)
        self._isdone = False

        if not sync:
            return

        try:
            timeout = _GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT * S_TO_NS
            start = self._node.get_clock().now()
            while not self._isdone:
                time_diff = self._node.get_clock().now() - start
                if time_diff.nanoseconds >= timeout:
                    self.cancel_goal()
                    raise exceptions.GripperError("Timed out")
            if not self._check_state(GoalStatus.STATUS_SUCCEEDED):
                raise exceptions.GripperError("Failed to apply force")
        except KeyboardInterrupt:
            self.cancel_goal()

    def _send_goal(self, client, goal):
        self.cancel_goal()
        self._send_goal_future = client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        self._current_client = client

    def _check_state(self, goal_status):
        if self._current_client is None:
            return False
        else:
            status = GoalStatus.STATUS_UNKNOWN
            if self._goal_handle is not None:
                status = self._goal_handle.status
            return status == goal_status

    def _feedback_callback(self, feedback_msg):
        self._state = feedback_msg.feedback.stalled

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._goal_handle = goal_handle
        future = goal_handle.get_result_async()
        future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        goal_handle = future.result()
        self._goal_handle = goal_handle
        self._isdone = True

    def is_moving(self):
        """Get the state as if the robot is moving.

        Returns:
            bool: True if the robot is moving
        """
        return self._check_state(GoalStatus.STATUS_EXECUTING)

    def is_succeeded(self):
        """Get the state as if the robot moving was succeeded.

        Returns:
            bool: True if success
        """
        return self._check_state(GoalStatus.STATUS_SUCCEEDED)

    def cancel_goal(self):
        """Cancel moving."""
        if not self.is_moving():
            return
        self._isdone = True
