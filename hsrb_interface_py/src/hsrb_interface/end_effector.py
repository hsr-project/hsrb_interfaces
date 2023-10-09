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

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from hsrb_interface.utils import CachingSubscriber
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal
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

_DISTANCE_MAX = (_PALM_TO_PROXIMAL_Y -
                 (_DISTAL_TO_TIP_Y *
                  math.cos(_DISTAL_JOINT_ANGLE_OFFSET) +
                  _DISTAL_TO_TIP_Z *
                  math.sin(_DISTAL_JOINT_ANGLE_OFFSET)) +
                 _PROXIMAL_TO_DISTAL_Z *
                 math.sin(_HAND_MOTOR_JOINT_MAX)) * 2
_DISTANCE_MIN = (_PALM_TO_PROXIMAL_Y -
                 (_DISTAL_TO_TIP_Y *
                  math.cos(_DISTAL_JOINT_ANGLE_OFFSET) +
                  _DISTAL_TO_TIP_Z *
                  math.sin(_DISTAL_JOINT_ANGLE_OFFSET)) +
                 _PROXIMAL_TO_DISTAL_Z *
                 math.sin(_HAND_MOTOR_JOINT_MIN)) * 2


class Gripper(robot.Item):
    """This class controls 2-finger gripper."""

    def __init__(self, name):
        """Initialize.

        Args:
            name (str): A name of a suction device file.
        """
        super(Gripper, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
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
        self._follow_joint_trajectory_client = actionlib.SimpleActionClient(
            prefix + "/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self._grasp_client = actionlib.SimpleActionClient(
            prefix + "/grasp",
            GripperApplyEffortAction
        )
        self._apply_force_client = actionlib.SimpleActionClient(
            prefix + "/apply_force",
            GripperApplyEffortAction
        )
        self._joint_state_sub = CachingSubscriber(
            "/hsrb/joint_states",
            JointState
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

               robot = hsrb_interface.Robot()
               gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
               gripper.command(1.2, 2.0)
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rospy.Duration(motion_time))
        ]

        self._send_goal(self._follow_joint_trajectory_client, goal)

        if not sync:
            return

        timeout = rospy.Duration(_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT)
        try:
            if self._follow_joint_trajectory_client.wait_for_result(timeout):
                s = self._follow_joint_trajectory_client.get_state()
                if s != actionlib.GoalStatus.SUCCEEDED:
                    msg = "Failed to follow commanded trajectory"
                    raise exceptions.GripperError(msg)
            else:
                self._follow_joint_trajectory_client.cancel_goal()
                raise exceptions.GripperError("Timed out")
        except KeyboardInterrupt:
            self._follow_joint_trajectory_client.cancel_goal()

    def get_distance(self):
        """Command get gripper finger tip distance.

        Returns:
            double: Distance between gripper finger tips [m]
        """
        joint_state = self._joint_state_sub.data
        hand_motor_pos = joint_state.position[
            joint_state.name.index(self._joint_names[0])]
        hand_left_position = joint_state.position[
            joint_state.name.index(
                self._left_finger_joint_name)] + hand_motor_pos
        hand_right_position = joint_state.position[
            joint_state.name.index(
                self._right_finger_joint_name)] + hand_motor_pos
        return ((math.sin(hand_left_position) +
                 math.sin(hand_right_position)) *
                _PROXIMAL_TO_DISTAL_Z +
                2 * (_PALM_TO_PROXIMAL_Y -
                     (_DISTAL_TO_TIP_Y *
                      math.cos(_DISTAL_JOINT_ANGLE_OFFSET) +
                      _DISTAL_TO_TIP_Z *
                      math.sin(_DISTAL_JOINT_ANGLE_OFFSET))))

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
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = self._joint_names
            goal.trajectory.points = [
                JointTrajectoryPoint(
                    time_from_start=rospy.Duration(1 / _DISTANCE_CONTROL_RATE))
            ]

            start_time = rospy.Time().now()
            elapsed_time = rospy.Duration(0.0)
            ierror = 0.0
            theta_ref = math.asin(((distance / 2 -
                                    (_PALM_TO_PROXIMAL_Y -
                                     (_DISTAL_TO_TIP_Y *
                                      math.cos(_DISTAL_JOINT_ANGLE_OFFSET) +
                                      _DISTAL_TO_TIP_Z *
                                      math.sin(_DISTAL_JOINT_ANGLE_OFFSET)))) /
                                   _PROXIMAL_TO_DISTAL_Z))
            rate = rospy.Rate(_DISTANCE_CONTROL_RATE)
            last_movement_time = rospy.Time.now()
            while elapsed_time.to_sec() < control_time:
                try:
                    error = distance - self.get_distance()
                    if abs(error) > _DISTANCE_CONTROL_STALL_THRESHOLD:
                        last_movement_time = rospy.Time.now()
                    if((rospy.Time.now() - last_movement_time).to_sec() >
                       _DISTANCE_CONTROL_STALL_TIMEOUT):
                        break
                    ierror += error
                    open_angle = (theta_ref +
                                  _DISTANCE_CONTROL_PGAIN * error +
                                  _DISTANCE_CONTROL_IGAIN * ierror)
                    goal.trajectory.points = [
                        JointTrajectoryPoint(
                            positions=[open_angle],
                            time_from_start=rospy.Duration(
                                _DISTANCE_CONTROL_TIME_FROM_START))
                    ]
                    self._follow_joint_trajectory_client.send_goal(goal)
                    elapsed_time = rospy.Time().now() - start_time
                except KeyboardInterrupt:
                    self._follow_joint_trajectory_client.cancel_goal()
                    return
                rate.sleep()

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
        goal = GripperApplyEffortGoal()
        goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        client = self._grasp_client
        if delicate:
            if effort < _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD:
                goal.effort = effort
                client = self._apply_force_client
            else:
                rospy.logwarn(
                    "Since effort is high, force control become invalid.")

        self._send_goal(client, goal)

        if not sync:
            return

        try:
            timeout = rospy.Duration(_GRIPPER_GRASP_TIMEOUT)
            if client.wait_for_result(timeout):
                client.get_result()
                state = client.get_state()
                if state != actionlib.GoalStatus.SUCCEEDED:
                    raise exceptions.GripperError("Failed to apply force")
            else:
                client.cancel_goal()
                raise exceptions.GripperError("Timed out")
        except KeyboardInterrupt:
            client.cancel_goal()

    def _send_goal(self, client, goal):
        self.cancel_goal()
        client.send_goal(goal)
        self._current_client = client

    def _check_state(self, goal_status):
        if self._current_client is None:
            return False
        else:
            state = self._current_client.get_state()
            return state == goal_status

    def is_moving(self):
        """Get the state as if the robot is moving.

        Returns:
            bool: True if the robot is moving
        """
        return self._check_state(actionlib.GoalStatus.ACTIVE)

    def is_succeeded(self):
        """Get the state as if the robot moving was succeeded.

        Returns:
            bool: True if success
        """
        return self._check_state(actionlib.GoalStatus.SUCCEEDED)

    def cancel_goal(self):
        """Cancel moving."""
        if not self.is_moving():
            return

        self._current_client.cancel_goal()


class Suction(object):
    """This class controls a suction nozzle.

    Args:
        name (str): A name of a suction device file.

    Returns:
        None
    """

    def __init__(self, name):
        """Initialize an instance with given parameters.

        Args:
            name (str):  A name of this resource
        """
        super(Suction, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
        if self._setting is None:
            msg = '{0}({1}) is not found '.format('end_effector', name)
            raise exceptions.ResourceNotFoundError(msg)
        self._name = name
        pub_topic_name = self._setting['suction_topic']
        self._pub = rospy.Publisher(pub_topic_name, Bool, queue_size=0)
        sub_topic_name = self._setting['pressure_sensor_topic']
        self._sub = utils.CachingSubscriber(sub_topic_name, Bool)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    def command(self, command):
        """Command on/off to a suction-nozzle.

        Args:
            command (bool): On if command is ``True``, Off otherwise

        Returns:
            None
        """
        if command < 0:
            msg = "'{0}' is not defined.".format(command)
            raise ValueError(msg)
        msg = Bool()
        msg.data = command
        self._pub.publish(msg)

    @property
    def pressure_sensor(self):
        """Get a sensor value (On/Off) of a suction-nozzle sensor.

        Returns:
            bool: True if ON.
        """
        return self._sub.data.data
