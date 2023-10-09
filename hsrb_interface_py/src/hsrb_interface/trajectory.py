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

"""This module classes and functions that manipulate joint trajectories"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
from itertools import repeat
import traceback

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
import rospy
import tf.transformations as T
from tmc_manipulation_msgs.msg import ArmNavigationErrorCodes
from tmc_manipulation_msgs.srv import (
    FilterJointTrajectory,
    FilterJointTrajectoryRequest,
    FilterJointTrajectoryWithConstraints,
    FilterJointTrajectoryWithConstraintsRequest,
    SelectConfig,
    SelectConfigRequest,
)
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from . import exceptions
from . import geometry
from . import settings
from . import utils


# Base frame of a mobile base in moition planning
_BASE_TRAJECTORY_ORIGIN = "odom"


def extract(trajectory, joint_names, joint_state):
    """Extract trajectories of specified joints from a given trajectory.

    If a given trajectory doesn't have a trajectory for a specified joint name,
    the trajctory of the joint is filled with a current joint state.

    Args:
        trajectory (trajectory_msgs.msg.JointTrajectory):
            A JointTrajectory to work on
        joint_names (List[str]):
            Target joint names
        joint_state (sensor_msgs.msg.JointState):
            A initial joint state to fill unspecified joint trajectory
    Returns:
        trajectory_msgs.msg.JointTrajectory: An extracted trajectory
    """
    num_points = len(trajectory.points)
    num_joints = len(joint_names)
    num_source_joints = len(trajectory.joint_names)
    index_map = list(repeat(0, num_joints))
    for joint_index in range(num_joints):
        index_map[joint_index] = -1
        for input_index in range(len(trajectory.joint_names)):
            if joint_names[joint_index] == trajectory.joint_names[input_index]:
                index_map[joint_index] = input_index
    trajectory_out = JointTrajectory()
    trajectory_out.joint_names = joint_names
    trajectory_out.points = list(utils.iterate(JointTrajectoryPoint,
                                               num_points))
    for point_index in range(num_points):
        target = trajectory_out.points[point_index]
        source = trajectory.points[point_index]
        target.positions = list(repeat(0, num_joints))
        target.velocities = list(repeat(0, num_joints))
        target.accelerations = list(repeat(0, num_joints))
        target.effort = list(repeat(0, num_joints))
        target.time_from_start = source.time_from_start
        # Check the point has enough elements
        # FIXME: If the given trajectory is well-formed, this check is not
        #        necessary. Actually we meet malformed trajectory sometime.
        has_velocities = (len(source.velocities) == num_source_joints)
        has_accelerations = (len(source.accelerations) == num_source_joints)
        has_effort = (len(source.effort) == num_source_joints)
        for joint_index in range(num_joints):
            if index_map[joint_index] != -1:
                pos = source.positions[index_map[joint_index]]
                target.positions[joint_index] = pos
                if has_velocities:
                    vel = source.velocities[index_map[joint_index]]
                    target.velocities[joint_index] = vel
                if has_accelerations:
                    acc = source.accelerations[index_map[joint_index]]
                    target.accelerations[joint_index] = acc
                if has_effort:
                    eff = source.effort[index_map[joint_index]]
                    target.effort[joint_index] = eff
            else:
                i = joint_state.name.index(joint_names[joint_index])
                angle = joint_state.position[i]
                target.positions[joint_index] = angle
                target.velocities[joint_index] = 0.0
                target.accelerations[joint_index] = 0.0
                target.effort[joint_index] = 0.0
    return trajectory_out


def merge(target, source):
    """Merge two trajectories into single trajectory.

    Those trajectories should have exactly same number of trajectory points.
    Result trajectory's ``time_from_start`` is set as same as `target` .

    Args:
        target(trajectory_msgs.msg.JointTrajectory):
            An original trajectory
        source(trajectory_msgs.msg.JointTrajectory):
            An additional trajectory
    Returns:
        trajectory_msgs.msg.JointTrajectory: A result trajectory
    Raises:
        ValueError: Two trajectories has different points size.
    """
    num_target_points = len(target.points)
    num_source_points = len(source.points)
    if num_target_points != num_target_points:
        msg = "Uneven trajectory size ({0} != {1})".format(num_target_points,
                                                           num_source_points)
        raise exceptions.TrajectoryLengthError(msg)
    merged = copy.deepcopy(target)
    merged.joint_names = list(merged.joint_names)
    merged.joint_names.extend(source.joint_names)

    num_points = len(merged.points)
    for i in range(num_points):
        merged.points[i].positions = list(merged.points[i].positions)
        merged.points[i].positions.extend(source.points[i].positions)
        merged.points[i].velocities = list(merged.points[i].velocities)
        merged.points[i].velocities.extend(source.points[i].velocities)
        merged.points[i].accelerations = list(merged.points[i].accelerations)
        merged.points[i].accelerations.extend(source.points[i].accelerations)
        merged.points[i].effort = list(merged.points[i].effort)
        merged.points[i].effort.extend(source.points[i].effort)
    return merged


def adjust_time(trajectory1, trajectory2):
    """Adjust time_from_start of trajectory points in trajectory2 to trajectory1.

    Velocities and accelerations are re-computed from differences.
    Two given trajectories must have exactly same timestamp.
    This function overwrite the `trajectory2` argument.

    Args:
        trajectory1 (trajectory_msgs.msg.JointTrajectory):
            Trajectory points in this trajectory is treated as base.
        trajectory2 (trajectory_msgs.msg.JointTrajectory):
            Trajectory points in this trajectory is adjusted to trajectory1.
    Returns:
        None
    """
    num_traj1_points = len(trajectory1.points)
    num_traj2_points = len(trajectory2.points)
    if num_traj1_points != num_traj2_points:
        msg = "Uneven trajectory size ({0} != {1})".format(num_traj1_points,
                                                           num_traj2_points)
        raise exceptions.TrajectoryLengthError(msg)
    num_points = len(trajectory1.points)
    # Adjust ``time_from_start`` of trajectory points in trajectory2 to
    # trajectory1
    for (point1, point2) in zip(trajectory1.points, trajectory2.points):
        point2.time_from_start = point1.time_from_start
    # Re-compute velocities in trajectory2 from difference of positions
    for index in range(num_points - 1):
        t_to = trajectory2.points[index + 1].time_from_start.to_sec()
        t_from = trajectory2.points[index].time_from_start.to_sec()
        dt = t_to - t_from
        p_to = trajectory2.points[index + 1].positions
        p_from = trajectory2.points[index].positions
        new_vels = [(x1 - x0) / dt for (x0, x1) in zip(p_from, p_to)]
        trajectory2.points[index].velocities = new_vels
    zero_vector2 = [0] * len(trajectory2.joint_names)
    trajectory2.points[-1].velocities = zero_vector2
    # Re-compute accelerations in trajectory2 from difference of velocties
    trajectory2.points[0].accelerations = zero_vector2
    for index in range(1, num_points - 1):
        t_to = trajectory2.points[index + 1].time_from_start.to_sec()
        t_from = trajectory2.points[index - 1].time_from_start.to_sec()
        dt = t_to - t_from
        p_to = trajectory2.points[index + 1].velocities
        p_from = trajectory2.points[index - 1].velocities
        new_accs = [(x1 - x0) / dt for (x0, x1) in zip(p_from, p_to)]
        trajectory2.points[index].accelerations = new_accs
    trajectory2.points[-1].accelerations = zero_vector2


def constraint_filter(joint_trajectory):
    """Apply joint constraint filter to an upper body trajectory.

    Args:
        joint_trajectory (trajectory_msgs.msg.JointTrajectory):
            A trajectory that will be applied this filter
    Returns:
        trajectory_msgs.msg.JointTrajectory:
            Filtered trajectory
    """
    service = settings.get_entry("trajectory", "constraint_filter_service")
    filter_service = rospy.ServiceProxy(
        service,
        FilterJointTrajectoryWithConstraints
    )
    req = FilterJointTrajectoryWithConstraintsRequest()
    req.trajectory = joint_trajectory
    timeout = settings.get_entry('trajectory', 'filter_timeout')
    req.allowed_time = rospy.Duration(timeout)
    try:
        res = filter_service.call(req)
        if res.error_code.val != ArmNavigationErrorCodes.SUCCESS:
            msg = "Failed to filter trajectory" + str(type(res.error_code))
            raise exceptions.TrajectoryFilterError(msg, res.error_code)
    except rospy.ServiceException:
        traceback.print_exc()
        raise
    return res.trajectory


def timeopt_filter(base_trajectory):
    """Apply timeopt filter to a omni-base trajectory.

    Args:
        joint_trajectory (trajectory_msgs.msg.JointTrajectory):
            A trajectory that will be applied this filter
    Returns:
        trajectory_msgs.msg.JointTrajectory:
            Filtered trajectory
    """
    service = settings.get_entry("trajectory", "timeopt_filter_service")
    filter_service = rospy.ServiceProxy(service, FilterJointTrajectory)
    req = FilterJointTrajectoryRequest()
    req.trajectory = base_trajectory
    try:
        res = filter_service.call(req)
        if res.error_code.val != ArmNavigationErrorCodes.SUCCESS:
            msg = "Failed to filter trajectory" + str(type(res.error_code))
            raise exceptions.TrajectoryFilterError(msg, res.error_code)
    except rospy.ServiceException:
        traceback.print_exc()
        raise
    filtered_traj = res.trajectory
    return filtered_traj


def hsr_timeopt_filter(merged_trajectory, start_state):
    """whole body timeopt filter.

    Args:
       merged_trajectory (trajectory_msgs.msg.JointTrajectory):
           A trajectory that will be applied this filter
       start_state: states
    Returns:
        trajectory_msgs.msg.JointTrajectory:
            Filtered trajectory
    """
    service = settings.get_entry("trajectory", "whole_timeopt_filter_service")
    caster_joint = settings.get_entry("trajectory", "caster_joint")
    filter_service = rospy.ServiceProxy(service, FilterJointTrajectory)
    req = FilterJointTrajectoryRequest()
    req.trajectory = merged_trajectory

    whole_name = merged_trajectory.joint_names + [caster_joint]
    req.start_state.joint_state.name = whole_name
    whole_pos = [start_state.position[start_state.name.index(joint)]
                 for joint in whole_name]
    req.start_state.joint_state.position = whole_pos

    try:
        res = filter_service.call(req)
        if res.error_code.val != ArmNavigationErrorCodes.SUCCESS:
            return None
    except rospy.ServiceException:
        traceback.print_exc()
        raise
    filtered_traj = res.trajectory
    return filtered_traj


def transform_base_trajectory(base_traj, tf2_buffer, tf_timeout, joint_names):
    """Transform a base trajectory to an ``odom`` frame based trajectory.

    Args:
        base_traj (trajectory_msgs.msg.MultiDOFJointTrajectory):
            A base trajectory
        tf2_buffer (tf2_ros.Buffer): Tf2 buffer
        tf_timeout (float): Timeout to get transform [sec]
        joint_names (list[str]):
            Joint names of [X-axis position, Y-axis position, Yaw position]
    Returns:
        trajectory_msgs.msg.JointTrajectory:
            A base trajectory based on ``odom`` frame.
    """
    odom_to_frame_transform = tf2_buffer.lookup_transform(
        _BASE_TRAJECTORY_ORIGIN,
        base_traj.header.frame_id,
        rospy.Time.now(),
        rospy.Duration(tf_timeout))
    odom_to_frame = geometry.transform_to_tuples(
        odom_to_frame_transform.transform)

    num_points = len(base_traj.points)
    odom_base_traj = JointTrajectory()
    odom_base_traj.points = list(utils.iterate(JointTrajectoryPoint,
                                               num_points))
    odom_base_traj.header = base_traj.header
    odom_base_traj.joint_names = joint_names

    # Transform each point into odom frame
    previous_theta = 0.0
    for i in range(num_points):
        t = base_traj.points[i].transforms[0]
        frame_to_base = geometry.transform_to_tuples(t)

        # odom_to_base = odom_to_frame * frame_to_base
        (odom_to_base_trans, odom_to_base_rot) = geometry.multiply_tuples(
            odom_to_frame, frame_to_base)
        odom_base_traj.points[i].positions = [odom_to_base_trans[0],
                                              odom_to_base_trans[1],
                                              0]
        roll, pitch, yaw = T.euler_from_quaternion(
            odom_to_base_rot)
        dtheta = geometry.shortest_angular_distance(previous_theta, yaw)
        theta = previous_theta + dtheta

        odom_base_traj.points[i].positions[2] = theta
        previous_theta = theta
    return odom_base_traj


class TrajectoryController(object):
    """Wrapper class for FollowJointTrajectoryAction

    Args:
        controller_name (str):
            A name of a ros-controls controller
        joint_names_suffix (str):
            A name of a parameter to specify target joint names

    Attributes:
        joint_names (List[str]): Names of target joints.
        controller_name (str): A name of a target controller.
    """

    def __init__(self, controller_name, joint_names_suffix="/joints"):
        """See class docstring."""
        self._controller_name = controller_name
        action = controller_name + "/follow_joint_trajectory"
        self._client = actionlib.SimpleActionClient(
            action,
            FollowJointTrajectoryAction
        )
        timeout = settings.get_entry('trajectory', 'action_timeout')
        self._client.wait_for_server(rospy.Duration(timeout))
        param_name = "{0}{1}".format(
            self._controller_name,
            joint_names_suffix
        )
        self._joint_names = rospy.get_param(param_name)

    def submit(self, trajectory):
        """Send a trajectory to a connecting controller."""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._client.send_goal(goal)

    def cancel(self):
        """Cancel a current goal."""
        self._client.cancel_goal()

    def get_state(self):
        """Get a status of the action client"""
        return self._client.get_state()

    def get_status_text(self):
        """Get a goal status text of the action client"""
        return self._client.get_goal_status_text()

    def get_result(self, timeout=None):
        """Get a result of a current goal.

        Returns:
            FollowJointTrajectoryResult: Execution result
        """
        if self._client.get_result() is None:
            return None

        state = self._client.get_state()
        result = self._client.get_result()

        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            msg = "{0}".format(result.error_code)
            raise exceptions.FollowTrajectoryError(msg)
        if state != actionlib.GoalStatus.SUCCEEDED:
            raise exceptions.FollowTrajectoryError("{0}".format(state))
        return result

    def _get_joint_names(self):
        return self._joint_names
    joint_names = property(_get_joint_names)

    def _get_controller_name(self):
        return self._controller_name
    controller_name = property(_get_controller_name)


class ImpedanceController(TrajectoryController):
    """A class to invoke impedance control action.

    Args:
        controller_name (str):
            A name of ros-controls controller
        joint_names_suffix (str):
            A name of a parameter to specify target joint names

    Attributes:
        config (str): A name of preset config.
        config_names (List[str]): A list of available preset configs.
    """

    def __init__(self, controller_name, joint_names_suffix="/joint_names"):
        """See class docstring."""
        super(ImpedanceController, self).__init__(controller_name,
                                                  joint_names_suffix)
        self._config = None
        self._config_names = rospy.get_param(controller_name + "/config_names",
                                             [])
        self._controller_name = controller_name

    def submit(self, trajectory):
        """Send a trajectory to a connecting controller"""
        if self._config is not None:
            setter_service = rospy.ServiceProxy(
                self._controller_name + "/select_config", SelectConfig)
            req = SelectConfigRequest()
            req.name = self._config
            try:
                res = setter_service.call(req)
                if not res.is_success:
                    msg = "Failed to set impedance config"
                    raise exceptions.FollowTrajectoryError(msg)
            except rospy.ServiceException:
                import traceback
                traceback.print_exc()
                raise
        else:
            msg = "Impedance config is None. But impedance control is called."
            raise exceptions.FollowTrajectoryError(msg)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._client.send_goal(goal)

    def _get_config(self):
        return self._config

    def _set_config(self, value):
        if value in self._config_names:
            self._config = value
        elif value is None:
            self._config = None
        else:
            msg = "Impedance config \"" + value + "\" is not determined."
            raise exceptions.FollowTrajectoryError(msg)

    config = property(_get_config, _set_config)

    def _get_config_names(self):
        self._config_names = rospy.get_param(
            self.controller_name + "/config_names", [])
        return self._config_names

    config_names = property(_get_config_names)


def wait_controllers(controllers):
    watch_rate = settings.get_entry('trajectory', 'watch_rate')
    rate = rospy.Rate(watch_rate)
    ok_set = {
        actionlib.GoalStatus.PENDING,
        actionlib.GoalStatus.ACTIVE,
        actionlib.GoalStatus.SUCCEEDED,
    }
    try:
        while True:
            states = [c.get_state() for c in controllers]
            if any(map(lambda s: s not in ok_set, states)):
                log = []
                for c in controllers:
                    log.append("{0}({1})".format(c.controller_name,
                                                 c.get_state()))
                    c.cancel()
                reason = ', '.join(log)
                text = "Playing trajectory failed: {0}".format(reason)
                raise exceptions.FollowTrajectoryError(text)
            if all([s == actionlib.GoalStatus.SUCCEEDED for s in states]):
                break
            rate.sleep()
    except KeyboardInterrupt:
        for c in controllers:
            c.cancel()
