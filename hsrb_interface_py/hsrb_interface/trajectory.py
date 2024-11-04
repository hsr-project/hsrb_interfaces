'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
# vim: fileencoding=utf-8
"""This module classes and functions that manipulate joint trajectories"""

import asyncio
import copy
from itertools import repeat
import time
import traceback

import action_msgs.msg as action_msgs
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
import tf_transformations as T
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint

from . import exceptions
from . import geometry
from . import robot
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
        target.positions = list(repeat(0.0, num_joints))
        target.velocities = list(repeat(0.0, num_joints))
        target.accelerations = list(repeat(0.0, num_joints))
        # Effort has not supported in ROS2
        # target.effort = list(repeat(0.0, num_joints))
        target.time_from_start = source.time_from_start
        # Check the point has enough elements
        # FIXME: If the given trajectory is well-formed, this check is not
        #        necessary. Actually we meet malformed trajectory sometime.
        has_velocities = (len(source.velocities) == num_source_joints)
        has_accelerations = (len(source.accelerations) == num_source_joints)
        # Effort has not supported in ROS2
        # has_effort = (len(source.effort) == num_source_joints)
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
                # Effort has not supported in ROS2
                # if has_effort:
                #    eff = source.effort[index_map[joint_index]]
                #    target.effort[joint_index] = eff
            else:
                i = joint_state.name.index(joint_names[joint_index])
                angle = joint_state.position[i]
                target.positions[joint_index] = angle
                target.velocities[joint_index] = 0.0
                target.accelerations[joint_index] = 0.0
                # Effort has not supported in ROS2
                # target.effort[joint_index] = 0.0
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


def timeopt_filter(base_trajectory, node):
    """Apply timeopt filter to a omni-base trajectory.

    Args:
        joint_trajectory (trajectory_msgs.msg.JointTrajectory):
            A trajectory that will be applied this filter
    Returns:
        trajectory_msgs.msg.JointTrajectory:
            Filtered trajectory
    """
    service = settings.get_entry("trajectory", "timeopt_filter_service")
    filter_service = node.create_client(FilterJointTrajectory, service)
    req = FilterJointTrajectory.Request()
    req.trajectory = base_trajectory
    try:
        future = filter_service.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            msg = "Failed to filter trajectory" + str(type(res.error_code))
            raise exceptions.TrajectoryFilterError(msg, res.error_code)
    except Exception:
        traceback.print_exc()
        raise
    filtered_traj = res.trajectory
    return filtered_traj


def hsr_timeopt_filter(merged_trajectory, start_state, node):
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
    filter_service = node.create_client(FilterJointTrajectory, service)
    req = FilterJointTrajectory.Request()
    req.trajectory = merged_trajectory

    whole_name = merged_trajectory.joint_names + [caster_joint]
    req.start_state.joint_state.name = whole_name
    whole_pos = [start_state.position[start_state.name.index(joint)]
                 for joint in whole_name]
    req.start_state.joint_state.position = whole_pos

    try:
        future = filter_service.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        res = future.result()
        if not res.is_success:
            return None
    except Exception:
        traceback.print_exc()
        raise
    filtered_traj = res.trajectory
    return filtered_traj


def transform_base_trajectory(
        base_traj,
        tf2_buffer,
        tf_timeout,
        joint_names,
        node):
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
    stamp = rclpy.time.Time()
    odom_to_frame_future = tf2_buffer.wait_for_transform_async(
        target_frame=_BASE_TRAJECTORY_ORIGIN,
        source_frame=base_traj.header.frame_id,
        time=stamp)
    rclpy.spin_until_future_complete(node, odom_to_frame_future, timeout_sec=tf_timeout)
    odom_to_frame_transform = asyncio.run(tf2_buffer.lookup_transform_async(
        _BASE_TRAJECTORY_ORIGIN, base_traj.header.frame_id, stamp))

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
        odom_base_traj.points[i].positions = [float(odom_to_base_trans[0]),
                                              float(odom_to_base_trans[1]),
                                              float(0.0)]
        roll, pitch, yaw = T.euler_from_quaternion(
            odom_to_base_rot)
        dtheta = geometry.shortest_angular_distance(previous_theta, yaw)
        theta = previous_theta + dtheta

        odom_base_traj.points[i].positions[2] = theta
        previous_theta = theta
    return odom_base_traj


class TrajectoryController(robot.Item):
    """Wrapper class for FollowJointTrajectory

    Args:
        controller_name (str):
            A name of a ros-controls controller
        joint_names_suffix (str):
            A name of a parameter to specify target joint names

    Attributes:
        joint_names (List[str]): Names of target joints.
        controller_name (str): A name of a target controller.
    """

    def __init__(self, controller_name, joint_names_suffix="joints"):
        super(TrajectoryController, self).__init__()
        """See class docstring."""
        self._controller_name = controller_name
        action = controller_name + "/follow_joint_trajectory"
        self._client = ActionClient(self._node, FollowJointTrajectory, action)
        timeout = settings.get_entry('trajectory', 'action_timeout')
        self._client.wait_for_server(timeout)
        param_name = "{0}".format(
            joint_names_suffix
        )
        res = utils.get_parameters_from_another_node(
            self._node, self._controller_name + '/get_parameters', [param_name])
        if len(res) != 0:
            self._joint_names = res[0]

    def submit(self, trajectory):
        """Send a trajectory to a connecting controller."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._send_goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self._node, self._send_goal_future, timeout_sec=1.0)

    def cancel(self):
        """Cancel a current goal."""
        goal_handle = self._send_goal_future.result()
        goal_handle.cancel_goal_async()

    def get_state(self):
        """Get a status of the action client"""
        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, get_result_future, timeout_sec=1.0)
        res = get_result_future.result()
        if res is None:
            return action_msgs.GoalStatus.STATUS_EXECUTING
        else:
            return res.status

    def get_status_text(self):
        """Get a goal status text of the action client"""
        status_strings = {
            action_msgs.GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",  # noqa
            action_msgs.GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",  # noqa
            action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
            action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
            action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
            action_msgs.GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",  # noqa
            action_msgs.GoalStatus.STATUS_ABORTED: "STATUS_ABORTED"  # noqa
        }
        future = self._client._async()
        result_status = future.result().status
        return status_strings[result_status]

    def get_result(self, timeout=None):
        """Get a result of a current goal.

        Returns:
            FollowJointTrajectoryResult: Execution result
        """
        goal_handle = self._send_goal_future.result()
        if goal_handle.get_result_async() is None:
            return None

        future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, future)
        result = future.result()
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            msg = "{0}".format(result.error_code)
            raise exceptions.FollowTrajectoryError(msg)
        if result.status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            raise exceptions.FollowTrajectoryError("{0}".format(result.status))
        return result

    def _get_joint_names(self):
        return self._joint_names
    joint_names = property(_get_joint_names)

    def _get_controller_name(self):
        return self._controller_name
    controller_name = property(_get_controller_name)


def wait_controllers(node, controllers):
    watch_rate = settings.get_entry('trajectory', 'watch_rate')
    ok_set = {
        action_msgs.GoalStatus.STATUS_UNKNOWN,
        action_msgs.GoalStatus.STATUS_EXECUTING,
        action_msgs.GoalStatus.STATUS_SUCCEEDED,
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
            if all([s == action_msgs.GoalStatus.STATUS_SUCCEEDED for s in states]):
                break
            time.sleep(float(1.0 / watch_rate))
    except KeyboardInterrupt:
        for c in controllers:
            c.cancel()
