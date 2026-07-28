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
"""Unittest for hsrb_interface.mobile_base module"""
from unittest.mock import MagicMock
from unittest.mock import patch
import warnings

import _testing as testing
import action_msgs.msg as action_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import hsrb_interface
from hsrb_interface import Robot

import hsrb_interface.mobile_base
from nav2_msgs.action import NavigateToPose
import rclpy
from sensor_msgs.msg import JointState
import tf_transformations
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class OmniBaseTest(testing.RosMockTestCase):

    def setUp(self):
        super().setUp()

        patcher = patch("hsrb_interface.trajectory.TrajectoryController")
        self.traj_controller_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.mobile_base.ActionClient")
        self.action_client_mock_cls = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.utils.get_transform")
        self.get_transform_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.trajectory.transform_base_trajectory")
        self.transform_base_trajectory_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.trajectory.hsr_timeopt_filter")
        self.hsr_timeopt_filter_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.trajectory.wait_controllers")
        self.wait_controllers_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_init(self):
        """Test MobileBase.__init__"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
        self.get_entry_mock.assert_called_with('mobile_base', 'omni_base')
        self.traj_controller_mock.assert_called_with('/omni_base_controller', 'base_coordinates')
        self.action_client_mock_cls.assert_called_with(mobile_base._node, NavigateToPose, '/move_base/move')
        self.caching_sub_mock.assert_called_with('/joint_states', JointState, default=JointState())

    def test_goto_x_y_yaw(self):
        """Test MobileBase.go_abs and MobileBase.go_rel"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_client_get_result_async = mock_client_send_goal.get_result_async.return_value

        mock_client_get_result = mock_client_get_result_async.result.return_value
        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_SUCCEEDED

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        # Test go_abs
        mobile_base.go_abs(0.0, 1.0, 2.0, timeout=3.0)

        expected_goal = NavigateToPose.Goal()
        expected_goal.pose.header.frame_id = 'test_frame'
        expected_goal.pose.header.stamp = rclpy.time.Time().to_msg()
        expected_goal.pose.pose.position.x = 0.0
        expected_goal.pose.pose.position.y = 1.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, 2.0)
        expected_goal.pose.pose.orientation.x = q[0]
        expected_goal.pose.pose.orientation.y = q[1]
        expected_goal.pose.pose.orientation.z = q[2]
        expected_goal.pose.pose.orientation.w = q[3]

        self.get_frame_mock.assert_called_with('map')
        self.action_client_mock_cls.return_value.send_goal_async.assert_called_with(expected_goal)

        # Test go_rel
        mobile_base.go_rel(0.0, 1.0, 2.0, timeout=3.0)

        self.get_frame_mock.assert_called_with('base')

        # Test deprecated warnings
        warnings.simplefilter('always')
        with warnings.catch_warnings(record=True) as w:
            mobile_base.go(0.0, 1.0, 2.0, timeout=3.0, relative=False)
            self.get_frame_mock.assert_called_with('map')
            self.assertEqual(w[0].category, hsrb_interface.exceptions.HsrbInterfaceDeprecationWarning)

        with warnings.catch_warnings(record=True) as w:
            mobile_base.go(0.0, 1.0, 2.0, timeout=3.0, relative=True)
            self.get_frame_mock.assert_called_with('base')
            self.assertEqual(w[0].category, hsrb_interface.exceptions.HsrbInterfaceDeprecationWarning)

    def test_goto_pos_ori(self):
        """Test MobileBase.go_pose"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_client_get_result_async = mock_client_send_goal.get_result_async.return_value

        mock_client_get_result = mock_client_get_result_async.result.return_value
        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_SUCCEEDED

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        pose = ((0.0, 1.0, 2.0), (0.5, 0.5, 0.5, 0.5))
        mobile_base.go_pose(pose, timeout=3.0, ref_frame_id='map')

        expected_goal = NavigateToPose.Goal()
        expected_goal.pose.header.frame_id = 'map'
        expected_goal.pose.header.stamp = rclpy.time.Time().to_msg()
        expected_goal.pose.pose.position.x = 0.0
        expected_goal.pose.pose.position.y = 1.0
        expected_goal.pose.pose.position.z = 2.0
        expected_goal.pose.pose.orientation.x = 0.5
        expected_goal.pose.pose.orientation.y = 0.5
        expected_goal.pose.pose.orientation.z = 0.5
        expected_goal.pose.pose.orientation.w = 0.5

        self.action_client_mock_cls.return_value.send_goal_async.assert_called_with(expected_goal)

        warnings.simplefilter('always')
        with warnings.catch_warnings(record=True) as w:
            mobile_base.move(pose, timeout=3.0, ref_frame_id='map')
            self.assertEqual(w[0].category, hsrb_interface.exceptions.HsrbInterfaceDeprecationWarning)

    def test_get_pose(self):
        """Test MobileBase.get_pose()"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.frame_id = 'test_origin'
        transform_stamped_msg.child_frame_id = 'test_frame'
        transform_stamped_msg.transform.translation.x = 1.0
        transform_stamped_msg.transform.translation.y = 2.0
        transform_stamped_msg.transform.translation.z = 3.0
        transform_stamped_msg.transform.rotation.x = 0.5
        transform_stamped_msg.transform.rotation.y = 0.5
        transform_stamped_msg.transform.rotation.z = 0.5
        transform_stamped_msg.transform.rotation.w = 0.5
        self.get_transform_mock.return_value = transform_stamped_msg

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        pose = mobile_base.get_pose('test_origin')
        self.assertEqual(pose, ((1.0, 2.0, 3.0), (0.5, 0.5, 0.5, 0.5)))

        self.get_frame_mock.assert_called_with('base')
        self.get_transform_mock.assert_called_with(
            mobile_base._node, mobile_base._tf2_buffer, 'test_origin', 'test_frame', 5.0)

    def test_go_failure(self):
        """Test MobileBase.go_abs and MobileBase.go_rel failure if timeout is invalid"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_client_get_result_async = mock_client_send_goal.get_result_async.return_value

        mock_client_get_result = mock_client_get_result_async.result.return_value
        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_SUCCEEDED

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
        mobile_base.go_abs(0.0, 1.0, 2.0, timeout=3.0)

        with self.assertRaises(ValueError):
            mobile_base.go_abs(0.0, 1.0, 2.0, timeout=-1.0)

        with self.assertRaises(ValueError):
            mobile_base.go_abs(0.0, 1.0, 2.0, timeout=float('inf'))

        with self.assertRaises(ValueError):
            mobile_base.go_abs(0.0, 1.0, 2.0, timeout=float('nan'))

        with self.assertRaises(ValueError):
            mobile_base.go_rel(0.0, 1.0, 2.0, timeout=-1.0)

        with self.assertRaises(ValueError):
            mobile_base.go_rel(0.0, 1.0, 2.0, timeout=float('inf'))

        with self.assertRaises(ValueError):
            mobile_base.go_rel(0.0, 1.0, 2.0, timeout=float('nan'))

    def test_follow_trajectory(self):
        """Test MobileBase.follow_trajectory()"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        trajectory = JointTrajectory()
        for _ in range(3):
            trajectory.points.append(JointTrajectoryPoint())
        self.transform_base_trajectory_mock.return_value = trajectory
        self.hsr_timeopt_filter_mock.return_value = trajectory

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        mobile_base.get_pose = MagicMock()
        mobile_base.get_pose.return_value = ((2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

        poses = [hsrb_interface.geometry.pose(x=1.0),
                 hsrb_interface.geometry.pose(x=0.0)]
        mobile_base.follow_trajectory(poses)

        self.get_frame_mock.assert_called_with('map')
        self.hsr_timeopt_filter_mock.assert_called_with(
            trajectory, mobile_base._joint_state_sub.data, mobile_base._node)
        self.traj_controller_mock.return_value.submit.assert_called_with(trajectory)
        self.wait_controllers_mock.assert_called_with(mobile_base._node, [self.traj_controller_mock.return_value])

        trajectory = self.transform_base_trajectory_mock.call_args[0][0]
        self.assertEqual(trajectory.header.frame_id, 'test_frame')
        self.assertEqual(len(trajectory.points), 3)
        self.assertAlmostEqual(trajectory.points[0].transforms[0].translation.x, 2.0)
        self.assertAlmostEqual(trajectory.points[1].transforms[0].translation.x, 1.0)
        self.assertAlmostEqual(trajectory.points[2].transforms[0].translation.x, 0.0)

        # Set ref_frame_id
        self.get_frame_mock.reset_mock()
        mobile_base.follow_trajectory(poses, ref_frame_id='var')

        self.get_frame_mock.assert_not_called()
        trajectory = self.transform_base_trajectory_mock.call_args[0][0]
        self.assertEqual(trajectory.header.frame_id, 'var')

    def test_follow_trajectory_with_stamp(self):
        """Test MobileBase.follow_trajectory() with stamped poses"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        trajectory = JointTrajectory()
        for _ in range(3):
            trajectory.points.append(JointTrajectoryPoint())
        self.transform_base_trajectory_mock.return_value = trajectory

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        mobile_base.get_pose = MagicMock()
        mobile_base.get_pose.return_value = ((2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

        poses = [hsrb_interface.geometry.pose(x=1.0),
                 hsrb_interface.geometry.pose(x=0.0)]
        time_from_starts = [3.0, 6.0]
        mobile_base.follow_trajectory(poses, time_from_starts)

        self.get_frame_mock.assert_called_with('map')
        self.hsr_timeopt_filter_mock.assert_not_called()

        trajectory = self.traj_controller_mock.return_value.submit.call_args[0][0]

        point_1 = trajectory.points[0]
        self.assertAlmostEqual(point_1.time_from_start.sec + point_1.time_from_start.nanosec * 1e-9, 3.0)

        point_2 = trajectory.points[1]
        self.assertAlmostEqual(point_2.time_from_start.sec + point_2.time_from_start.nanosec * 1e-9, 6.0)

        # Length of time_from_starts and poses should be same
        with self.assertRaises(ValueError):
            mobile_base.follow_trajectory(poses, [3.0])

        with self.assertRaises(ValueError):
            mobile_base.follow_trajectory(poses, [0.0, 3.0, 6.0])

    def test_create_go_pose_goal(self):
        """Test MobileBase.create_go_pose_goal"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        goal = mobile_base.create_go_pose_goal(hsrb_interface.geometry.pose(x=1.0))
        self.assertEqual(goal.header.frame_id, 'test_frame')
        self.assertAlmostEqual(goal.pose.position.x, 1.0)

        goal = mobile_base.create_go_pose_goal(hsrb_interface.geometry.pose(),
                                               ref_frame_id='piyo')
        self.assertEqual(goal.header.frame_id, 'piyo')
        self.assertAlmostEqual(goal.pose.position.x, 0.0)

    def test_create_follow_goal(self):
        """Test MobileBase.create_follow_trajectory_goal"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }
        self.get_frame_mock.return_value = 'test_frame'

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        mobile_base.get_pose = MagicMock()
        mobile_base.get_pose.return_value = ((2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

        poses = [hsrb_interface.geometry.pose(x=1.0),
                 hsrb_interface.geometry.pose(x=0.0)]
        _ = mobile_base.create_follow_trajectory_goal(poses)

        self.get_frame_mock.assert_called_with('map')
        trajectory = self.transform_base_trajectory_mock.call_args[0][0]
        self.assertEqual(trajectory.header.frame_id, 'test_frame')
        self.assertEqual(len(trajectory.points), 3)
        self.assertAlmostEqual(trajectory.points[0].transforms[0].translation.x, 2.0)
        self.assertAlmostEqual(trajectory.points[1].transforms[0].translation.x, 1.0)
        self.assertAlmostEqual(trajectory.points[2].transforms[0].translation.x, 0.0)

        # Set ref_frame_id
        self.get_frame_mock.reset_mock()
        _ = mobile_base.create_follow_trajectory_goal(poses, ref_frame_id='var')
        self.get_frame_mock.assert_not_called()
        trajectory = self.transform_base_trajectory_mock.call_args[0][0]
        self.assertEqual(trajectory.header.frame_id, 'var')

        # With time_from_starts
        self.get_frame_mock.reset_mock()
        trajectory = JointTrajectory()
        for _ in range(3):
            trajectory.points.append(JointTrajectoryPoint())
        self.transform_base_trajectory_mock.return_value = trajectory
        goal = mobile_base.create_follow_trajectory_goal(poses, [3.0, 6.0])

        self.get_frame_mock.assert_called_with('map')
        self.assertEqual(len(goal.points), 2)
        self.assertAlmostEqual(goal.points[0].time_from_start.sec + goal.points[0].time_from_start.nanosec * 1e-9, 3.0)
        self.assertAlmostEqual(goal.points[1].time_from_start.sec + goal.points[1].time_from_start.nanosec * 1e-9, 6.0)

    def test_execute(self):
        """Test MobileBase.execute()"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }

        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_client_get_result_async = mock_client_send_goal.get_result_async.return_value

        mock_client_get_result = mock_client_get_result_async.result.return_value
        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_SUCCEEDED

        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

        input_goal = PoseStamped()
        mobile_base.execute(input_goal)
        action_goal = mock_action_client.send_goal_async.call_args[0][0]
        self.assertEqual(action_goal.pose, input_goal)

        mobile_base.execute(JointTrajectory())
        self.traj_controller_mock.return_value.submit.assert_called_with(JointTrajectory())

        with self.assertRaises(ValueError):
            mobile_base.execute('hoge')

    def test_is_moving(self):
        """Test MobileBase.is_moving()"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }

        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_client_get_result_async = mock_client_send_goal.get_result_async.return_value
        mock_client_get_result = mock_client_get_result_async.result.return_value
        mock_follow_client = self.traj_controller_mock.return_value

        # mobile_base.execute is not called
        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
        self.assertFalse(mobile_base.is_moving())

        # Send pose
        mobile_base.execute(PoseStamped())
        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_EXECUTING
        self.assertTrue(mobile_base.is_moving())

        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
        self.assertFalse(mobile_base.is_moving())

        # Send trajetory
        mobile_base.execute(JointTrajectory())
        mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_EXECUTING
        self.assertTrue(mobile_base.is_moving())

        mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_SUCCEEDED
        self.assertFalse(mobile_base.is_moving())

    def test_is_succeeded(self):
        """Test MobileBase.is_succeeded()"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }

        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_follow_client = self.traj_controller_mock.return_value

        # mobile_base.execute is not called
        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
        self.assertFalse(mobile_base.is_succeeded())

        # Send pose
        mobile_base.execute(PoseStamped())
        mock_client_send_goal.status = action_msgs.GoalStatus.STATUS_EXECUTING
        self.assertFalse(mobile_base.is_succeeded())

        mock_client_send_goal.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
        self.assertTrue(mobile_base.is_succeeded())

        # Send trajetory
        mobile_base.execute(JointTrajectory())
        mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_EXECUTING
        self.assertFalse(mobile_base.is_succeeded())

        mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_SUCCEEDED
        self.assertTrue(mobile_base.is_succeeded())

    def test_cancel_goal(self):
        """Test MobileBase.cancel_goal"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'navigation_action': '/move_base/move',
            'follow_trajectory_action': '/omni_base_controller',
            'pose_topic': '/global_pose',
            'joint_states_topic': '/joint_states',
        }

        mock_action_client = self.action_client_mock_cls.return_value
        mock_client_send_goal_async = mock_action_client.send_goal_async.return_value
        mock_client_send_goal = mock_client_send_goal_async.result.return_value
        mock_client_get_result_async = mock_client_send_goal.get_result_async.return_value
        mock_client_get_result = mock_client_get_result_async.result.return_value
        mock_follow_client = self.traj_controller_mock.return_value

        # Cancel without goal
        mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
        mobile_base.cancel_goal()

        mock_action_client.cancel_goal.assert_not_called()
        mock_follow_client.cancel.assert_not_called()

        # Send pose and cancel
        mobile_base.execute(PoseStamped())
        mock_client_get_result.status = action_msgs.GoalStatus.STATUS_EXECUTING
        mobile_base.cancel_goal()

        mock_client_send_goal.cancel_goal_async.assert_called_once_with()
        mock_follow_client.cancel.assert_not_called()

        # Send trajectory and cancel
        mobile_base.execute(JointTrajectory())
        mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_EXECUTING
        mobile_base.cancel_goal()

        mock_follow_client.cancel.assert_called_once_with()
