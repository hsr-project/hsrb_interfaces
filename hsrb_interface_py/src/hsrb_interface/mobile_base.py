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

"""Provide abstract inteface for a mobile base."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import math
import warnings

import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import rospy
import tf
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

from . import exceptions
from . import geometry
from . import robot
from . import settings
from . import trajectory
from . import utils

# Timeout to receve enough tf transform [sec]
_TF_TIMEOUT = 5.0

_ACTION_WAIT_TIMEOUT = 30.0


def _validate_timeout(timeout):
    """Validate a given timeout value is meaning time value."""
    if timeout < 0.0 or math.isnan(timeout) or math.isinf(timeout):
        raise ValueError("Invalid timeout: {0}".format(timeout))


class MobileBase(robot.Item):
    """Abstract interface to control a mobile base.

    Example:

        .. sourcecode:: python

           with hsrb_interface.Robot() as robot:
               omni_base = robot.get('omni_base')
               omni_base.go(1, 2, math.pi / 2.0)
               print(omni_base.pose)
    """

    def __init__(self, name):
        """Initialize an instance with a resource which has given `name`.

        Args:
            name (str): A name of a target resource.
        """
        super(MobileBase, self).__init__()
        self._setting = settings.get_entry('mobile_base', name)

        self._tf2_buffer = robot._get_tf2_buffer()

        action_name = self._setting['move_base_action']
        self._action_client = actionlib.SimpleActionClient(action_name,
                                                           MoveBaseAction)
        self._action_client.wait_for_server(
            rospy.Duration(_ACTION_WAIT_TIMEOUT))

        self._follow_client = trajectory.TrajectoryController(
            self._setting['follow_trajectory_action'], '/base_coordinates')

        self._current_client = None

    def go_pose(self, pose=geometry.pose(), timeout=0.0, ref_frame_id=None):
        """Move to a specified pose.

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            timeout (float): Timeout to movement [sec].
                If not specified, deafult is 0 and wait forever.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.try_get('omni_base')
                   pose = (Vector3(0.1, 0.2, 0.0), Quaternion())
                   base.go_pose(pose, 10.0, 'base_footprint')
        """
        _validate_timeout(timeout)
        goal = self.create_go_pose_goal(pose, ref_frame_id)
        self._send_goal_pose_and_wait(goal, timeout)

    def go_rel(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0):
        """Move base from current position.

        Args:
            x   (float): X-axis position on ``robot`` frame [m]
            y   (float): Y-axis position on ``robot`` frame [m]
            yaw (float): Yaw position on ``robot`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.try_get('omni_base')
                   base.go_rel(1.0, 0.0, 0.0)
        """
        _validate_timeout(timeout)
        pose = geometry.pose(x, y, 0.0, 0.0, 0.0, yaw)
        ref_frame_id = settings.get_frame('base')
        goal = self.create_go_pose_goal(pose, ref_frame_id)
        self._send_goal_pose_and_wait(goal, timeout)

    def go_abs(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0):
        """Move to a specified pose on map.

        Args:
            x   (float): X-axis position on ``map`` frame [m]
            y   (float): Y-axis position on ``map`` frame [m]
            yaw (float): Yaw position on ``map`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.try_get('omni_base')
                   base.go_abs(1.0, 0.0, 0.0)
        """
        _validate_timeout(timeout)
        pose = geometry.pose(x, y, 0.0, 0.0, 0.0, yaw)
        ref_frame_id = settings.get_frame('map')
        goal = self.create_go_pose_goal(pose, ref_frame_id)
        self._send_goal_pose_and_wait(goal, timeout)

    def move(self, pose, timeout=0.0, ref_frame_id=None):
        """Move to a specified pose.

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            timeout (float): Timeout to movement [sec].
                If not specified, deafult is 0 and wait forever.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.

        Warning:
            This function is deprecated. Use :py:func:`go_pose()` instead.
        """
        msg = ' '.join(["MobileBase.move() is depreacated."
                        "Use MobileBase.go_pose() instead."])
        warnings.warn(msg, exceptions.DeprecationWarning)
        self.go_pose(pose, timeout, ref_frame_id)

    def go(self, x, y, yaw, timeout=0.0, relative=False):
        """Move base to a specified pose.

        Args:
            x   (float): X-axis position on ``map`` frame [m]
            y   (float): Y-axis position on ``map`` frame [m]
            yaw (float): Yaw position on ``map`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.
            relative (bool): If ``True``, a robot move on robot frame.
                Otherwise a robot move on ``map`` frame.

        Warning:
            This function is deprecated.
            Use :py:func:`go_rel()` or :py:func:`go_abs()` instead.
        """
        msg = ' '.join(["MobileBase.go() is depreacated."
                        "Use MobileBase.go_rel() or"
                        "MobileBase.go_abs() instead."])
        warnings.warn(msg, exceptions.DeprecationWarning)
        if relative:
            self.go_rel(x, y, yaw, timeout)
        else:
            self.go_abs(x, y, yaw, timeout)

    def _send_goal_pose_and_wait(self, goal, timeout=0.0):
        self.execute(goal)
        try:
            if self._action_client.wait_for_result(rospy.Duration(timeout)):
                state = self._action_client.get_state()
                if state != actionlib.GoalStatus.SUCCEEDED:
                    error_text = self._action_client.get_goal_status_text()
                    msg = 'Failed to reach goal ({0})'.format(error_text)
                    raise exceptions.MobileBaseError(msg)
            else:
                self._action_client.cancel_goal()
                raise exceptions.MobileBaseError('Timed out')
        except KeyboardInterrupt:
            self._action_client.cancel_goal()

    def follow_trajectory(self, poses, time_from_starts=[], ref_frame_id=None):
        """Follow given poses and timing with ignoring the map.

        Args:
            poses (List[Tuple[Vector3, Quaternion]]):
                Target poses of the robot base.
            time_from_starts (List[float]):
                Times of each "poses" [sec]. If empty, the times are optimized
                by time-optimal trajectory filter.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.
        Returns:
            None

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   omni_base = robot.try_get('omni_base')
                   poses = [geometry.pose(x=1.0, y=0.0, ek=0.0),
                            geometry.pose(x=1.0, y=1.0, ek=math.pi)]
                   omni_base.follow_trajectory(poses)
        """
        goal = self.create_follow_trajectory_goal(
            poses, time_from_starts, ref_frame_id)
        self.execute(goal)
        trajectory.wait_controllers([self._follow_client])

    @property
    def pose(self):
        """Estimated pose of a robot on ``map`` frame.

        Returns:
            List[float]: A pose value structured as (x[m], y[m], yaw[rad]).
        """
        pos, ori = self.get_pose()
        quat = [ori.x, ori.y, ori.z, ori.w]
        euler_angles = tf.transformations.euler_from_quaternion(quat)
        yaw = euler_angles[2]
        return [pos.x, pos.y, yaw]

    def get_pose(self, ref_frame_id=None):
        """Get estimated pose of a robot on ``ref_frame_id`` frame.

        Args:
             ref_frame_id (str):
                 A reference frame of estimated pose. (Default ``map`` frame)
        Returns:
             Tuple[Vector3, Quaternion]:
                 A pose of ``base_footprint`` frame from ``ref_frame_id``.
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        trans = self._tf2_buffer.lookup_transform(ref_frame_id,
                                                  settings.get_frame('base'),
                                                  rospy.Time.now(),
                                                  rospy.Duration(_TF_TIMEOUT))
        return geometry.transform_to_tuples(trans.transform)

    def create_go_pose_goal(self, pose, ref_frame_id=None):
        """Create goal pose to move to a specified pose

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.
        Returns:
            geometry_msgs.msg.PoseStamped: A goal pose
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        target_pose = PoseStamped()
        target_pose.header.frame_id = ref_frame_id
        target_pose.header.stamp = rospy.Time(0)
        target_pose.pose = geometry.tuples_to_pose(pose)
        return target_pose

    def create_follow_trajectory_goal(self, poses,
                                      time_from_starts=[], ref_frame_id=None):
        """Create trajectory to follow given poses and timing.

        Args:
            poses (List[Tuple[Vector3, Quaternion]]):
                Target poses of the robot base.
            time_from_starts (List[float]):
                Times of each "poses" [sec]. If empty, the times are optimized
                by time-optimal trajectory filter.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.
        Returns:
            trajectory_msgs.msg.JointTrajectory: A base trajectory
        """
        num_poses = len(poses)
        num_times = len(time_from_starts)
        if (num_times != 0) and (num_poses != num_times):
            raise ValueError("The size of time_from_starts should be zero"
                             " or same as the size of poses")
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        ref_to_current = self.get_pose(ref_frame_id)
        input_trajectory = MultiDOFJointTrajectory()
        input_trajectory.header.frame_id = ref_frame_id
        input_trajectory.points = list(
            utils.iterate(MultiDOFJointTrajectoryPoint, num_poses + 1))
        input_trajectory.points[0].transforms.append(
            geometry.tuples_to_transform(ref_to_current))
        for index in range(num_poses):
            input_trajectory.points[index + 1].transforms.append(
                geometry.tuples_to_transform(poses[index]))

        transformed_trajectory = trajectory.transform_base_trajectory(
            input_trajectory, self._tf2_buffer, _TF_TIMEOUT,
            self._follow_client.joint_names)

        if num_times == 0:
            # TODO(Keisuke Takeshita): Use hsr_timeopt_filter
            base_trajectory = trajectory.timeopt_filter(
                transformed_trajectory)
            base_trajectory.header.stamp = rospy.Time(0)
        else:
            base_trajectory = transformed_trajectory
            for index in range(num_times):
                tfs = rospy.Duration(time_from_starts[index])
                base_trajectory.points[index + 1].time_from_start = tfs
        # Remove current point
        del base_trajectory.points[0]
        return base_trajectory

    def execute(self, goal):
        """Send a goal and not wait the result.

        Args:
            goal (geometry_msgs.msg.PoseStamped or
                  trajectory_msgs.msg.JointTrajectory): A goal to move

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   omni_base = robot.try_get('omni_base')
                   poses = [geometry.pose(x=1.0, y=0.0, ek=0.0),
                            geometry.pose(x=1.0, y=1.0, ek=math.pi)]
                   goal = omni_base.create_follow_trajectory_goal(poses)
                   omni_base.execute(goal)
                   while not rospy.is_shutdown():
                       rospy.sleep(1.0)
                       if not omni_base.is_moving():
                           break
                   print("Result: " + str(omni_base.is_succeeded())
        """
        self.cancel_goal()
        if isinstance(goal, PoseStamped):
            action_goal = MoveBaseGoal()
            action_goal.target_pose = goal
            self._action_client.send_goal(action_goal)
            self._current_client = self._action_client
        elif isinstance(goal, JointTrajectory):
            self._follow_client.submit(goal)
            self._current_client = self._follow_client
        else:
            raise ValueError("Invalid goal.")

    def is_moving(self):
        """Get the state as if the robot is moving.

        Returns:
            bool: True if the robot is moving
        """
        if self._current_client is None:
            return False
        else:
            state = self._current_client.get_state()
            return state == actionlib.GoalStatus.ACTIVE

    def is_succeeded(self):
        """Get the state as if the robot moving was succeeded.

        Returns:
            bool: True if success
        """
        if self._current_client is None:
            return False
        else:
            state = self._current_client.get_state()
            return state == actionlib.GoalStatus.SUCCEEDED

    def cancel_goal(self):
        """Cancel moving."""
        if not self.is_moving():
            return

        if self._current_client is self._action_client:
            self._action_client.cancel_goal()
        elif self._current_client is self._follow_client:
            self._follow_client.cancel()
