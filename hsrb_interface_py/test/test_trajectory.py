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

"""Unittest for hsrb_interface.trajectory module"""
from __future__ import absolute_import

import copy
import os
import sys
from unittest.mock import call, MagicMock, patch, PropertyMock

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from hsrb_interface_py import _testing as testing
from hsrb_interface_py import trajectory
from nose.tools import eq_
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class TrajectoryTestCase(testing.RosMockTestCase):

    def setUp(self):
        super(TrajectoryTestCase, self).setUp()

        patcher = patch("threading.Event.wait")
        self.event_wait_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def trajectory_fixture(self):
        """Create a example joint trajectory"""
        traj = JointTrajectory()
        traj.joint_names = ['a', 'b', 'c']
        point1 = JointTrajectoryPoint(positions=[0, 0, 0],
                                      velocities=[0, 0, 0],
                                      accelerations=[0, 0, 0],
                                      effort=[0, 0, 0],
                                      time_from_start=Duration(seconds=0).to_msg())
        point2 = JointTrajectoryPoint(positions=[1, 2, 3],
                                      velocities=[1, 2, 3],
                                      accelerations=[1, 2, 3],
                                      effort=[1, 2, 3],
                                      time_from_start=Duration(seconds=1).to_msg())
        point3 = JointTrajectoryPoint(positions=[2, 4, 6],
                                      velocities=[2, 4, 6],
                                      accelerations=[2, 4, 6],
                                      effort=[2, 4, 6],
                                      time_from_start=Duration(seconds=2).to_msg())
        traj.points.append(point1)
        traj.points.append(point2)
        traj.points.append(point3)
        return traj

    def state_fixture(self):
        """Create a example joint state"""
        state = JointState()
        state.name = ['a', 'b', 'c']
        state.position = [0.0, 0.0, 0.0]
        state.velocity = [1.0, 2.0, 3.0]
        state.effort = [4.0, 5.0, 6.0]
        return state


class TrajectoryModuleTest(TrajectoryTestCase):

    def test_extract_ok(self):
        """Test hsrb_interface.trajectory.extract()"""
        traj = self.trajectory_fixture()
        joint_state = JointState()
        joint_state.name = ['d']
        joint_state.position = [42.0]
        joint_state.velocity = [43.0]
        joint_state.effort = [44.0]
        result = trajectory.extract(traj, ['a', 'd'], joint_state)

        eq_(['a', 'd'], result.joint_names)
        eq_([0.0, 42.0], result.points[0].positions.tolist())
        eq_([0.0, 0.0], result.points[0].velocities.tolist())
        eq_([0.0, 0.0], result.points[0].accelerations.tolist())
        eq_([0.0, 0.0], result.points[0].effort.tolist())
        eq_(Duration(seconds=0).to_msg(), result.points[0].time_from_start)

        eq_([1.0, 42.0], result.points[1].positions.tolist())
        eq_([1.0, 0.0], result.points[1].velocities.tolist())
        eq_([1.0, 0.0], result.points[1].accelerations.tolist())
        eq_([1.0, 0.0], result.points[1].effort.tolist())
        eq_(Duration(seconds=1).to_msg(), result.points[1].time_from_start)

        eq_([2.0, 42.0], result.points[2].positions.tolist())
        eq_([2.0, 0.0], result.points[2].velocities.tolist())
        eq_([2.0, 0.0], result.points[2].accelerations.tolist())
        eq_([2.0, 0.0], result.points[2].effort.tolist())
        eq_(Duration(seconds=2).to_msg(), result.points[2].time_from_start)

    def test_merge_ok(self):
        """Test hsrb_interface.trajectory.merge()"""
        traj1 = JointTrajectory()
        traj1.joint_names = ['a', 'b']
        traj1.points = [
            JointTrajectoryPoint(positions=[0, 0],
                                 velocities=[0, 0],
                                 accelerations=[0, 0],
                                 effort=[0, 0],
                                 time_from_start=Duration(seconds=0).to_msg()),
            JointTrajectoryPoint(positions=[1, 2],
                                 velocities=[1, 2],
                                 accelerations=[1, 2],
                                 effort=[1, 2],
                                 time_from_start=Duration(seconds=1).to_msg()),
        ]
        traj2 = JointTrajectory()
        traj2.joint_names = ['c', 'd']
        traj2.points = [
            JointTrajectoryPoint(positions=[0, 0],
                                 velocities=[0, 0],
                                 accelerations=[0, 0],
                                 effort=[0, 0],
                                 time_from_start=Duration(seconds=3).to_msg()),
            JointTrajectoryPoint(positions=[3, 4],
                                 velocities=[3, 4],
                                 accelerations=[3, 4],
                                 effort=[3, 4],
                                 time_from_start=Duration(seconds=4).to_msg()),
        ]
        result = trajectory.merge(traj1, traj2)

        eq_(['a', 'b', 'c', 'd'], result.joint_names)
        eq_([0.0, 0.0, 0.0, 0.0], result.points[0].positions.tolist())
        eq_([0.0, 0.0, 0.0, 0.0], result.points[0].velocities.tolist())
        eq_([0.0, 0.0, 0.0, 0.0], result.points[0].accelerations.tolist())
        eq_([0.0, 0.0, 0.0, 0.0], result.points[0].effort.tolist())
        eq_(Duration(seconds=0).to_msg(), result.points[0].time_from_start)
        eq_([1.0, 2.0, 3.0, 4.0], result.points[1].positions.tolist())
        eq_([1.0, 2.0, 3.0, 4.0], result.points[1].velocities.tolist())
        eq_([1.0, 2.0, 3.0, 4.0], result.points[1].accelerations.tolist())
        eq_([1.0, 2.0, 3.0, 4.0], result.points[1].effort.tolist())
        eq_(Duration(seconds=1).to_msg(), result.points[1].time_from_start)

    def test_hsr_timeopt_filter_ok(self):
        """Test hsrb_interface.trajectory.hsr_timeopt_filter()"""
        # Setup pre-conditions
        self.get_entry_mock.side_effect = ['/timeopt_filter', 'a']
        service_client_mock = self.connection_mock.create_client()
        result = service_client_mock.call_async().result.return_value
        result.is_success = True
        traj = self.trajectory_fixture()
        state = self.state_fixture()

        # Call the target method
        trajectory.hsr_timeopt_filter(traj, state, self.connection_mock)

        # Check post-conditions
        self.get_entry_mock.assert_has_calls([
            call('trajectory', 'whole_timeopt_filter_service'),
            call('trajectory', 'caster_joint')
        ])
        self.connection_mock.create_client.assert_called_with(
            FilterJointTrajectory,
            "/timeopt_filter")
        req = FilterJointTrajectory.Request()
        req.trajectory = traj
        whole_name = traj.joint_names + ['a']
        req.start_state.joint_state.name = whole_name
        whole_pos = [state.position[state.name.index(joint)]
                     for joint in whole_name]
        req.start_state.joint_state.position = whole_pos

        service_client_mock.call_async.assert_called_with(req)


class TrajectoryControllerTest(TrajectoryTestCase):

    def test_creation_ok(self):
        # Create an instance of target class
        trajectory.TrajectoryController("test", self.connection_mock)
        req = GetParameters.Request()
        req.names = ["joints"]
        self.action_client_mock.assert_called_with(
            self.connection_mock,
            FollowJointTrajectory,
            "test/follow_joint_trajectory")
        self.get_entry_mock.assert_called_with('trajectory', 'action_timeout')
        self.connection_mock.create_client.ssert_called_with(
            GetParameters, "test/get_parameters")
        self.connection_mock.create_client().call_async.assert_called_with(req)

    def test_submit_ok(self):
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("test",
                                                     self.connection_mock)
        traj = self.trajectory_fixture()
        controller.submit(traj)

        # Check post-conditions
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        action_client_mock.send_goal_async.assert_called_with(goal)

    def test_cancel_ok(self):
        goal_handle_mock = MagicMock()
        controller = trajectory.TrajectoryController("test",
                                                     self.connection_mock)
        self.trajectory_fixture()
        controller._goal_handle = goal_handle_mock
        controller.cancel()

        # Check post-conditions
        goal_handle_mock.cancel_goal.assert_called_with()

    def test_get_state_ok(self):
        goal_handle_mock = MagicMock()
        status_mock = PropertyMock(return_value=GoalStatus.STATUS_SUCCEEDED)
        type(goal_handle_mock).status = status_mock
        controller = trajectory.TrajectoryController("test",
                                                     self.connection_mock)
        self.trajectory_fixture()
        controller._goal_handle = goal_handle_mock
        controller.get_state()

        # Check post-conditions
        eq_(GoalStatus.STATUS_SUCCEEDED, goal_handle_mock.status)

    def test_get_result_ok(self):
        expected_result = FollowJointTrajectory.Result()
        expected_result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle_mock = MagicMock()
        status_mock = PropertyMock(return_value=GoalStatus.STATUS_SUCCEEDED)
        type(goal_handle_mock).status = status_mock
        goal_handle_mock.get_result.return_value = copy.deepcopy(
            expected_result)

        controller = trajectory.TrajectoryController("test",
                                                     self.connection_mock)
        self.trajectory_fixture()
        controller._goal_handle = goal_handle_mock
        result = controller.get_result()

        # Check post-conditions
        eq_(GoalStatus.STATUS_SUCCEEDED, goal_handle_mock.status)
        goal_handle_mock.get_result.assert_called_with()
        eq_(expected_result, result)
