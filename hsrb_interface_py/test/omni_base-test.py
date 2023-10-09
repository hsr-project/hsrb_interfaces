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

"""Unittest for hsrb_interface.mobile_base module"""
import warnings

import actionlib
from geometry_msgs.msg import PoseStamped
import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.geometry
import hsrb_interface.mobile_base
from mock import MagicMock
from mock import patch
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from nose.tools import assert_almost_equal
from nose.tools import assert_false
from nose.tools import assert_raises
from nose.tools import assert_true
from nose.tools import eq_
from nose.tools import ok_
import tf
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base(mock_get_entry, mock_connecting,
                     mock_action_client_cls, mock_trajectory_controller):
    """Test simple use case of MobileBase class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    ok_(mobile_base)
    mock_get_entry.call_with_args("mobile_base", "omni_base")
    mock_action_client_cls.call_with_args("/move_base", MoveBaseAction)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rospy.Duration")
@patch("rospy.Time")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_x_y_yaw(mock_get_entry, mock_get_frame,
                                  mock_connecting,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):
    """Test MobileBase.go_abs and MobileBase.go_rel"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_get_frame.return_value = "hoge"
    mock_action_client = mock_action_client_cls.return_value
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    mock_action_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED
    mobile_base.go_abs(0, 1, 2, timeout=3.0)

    expected_goal = MoveBaseGoal()
    expected_goal.target_pose.header.frame_id = "hoge"
    expected_goal.target_pose.header.stamp = mock_time_cls(0)
    expected_goal.target_pose.pose.position.x = 0
    expected_goal.target_pose.pose.position.y = 1
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, 2)
    expected_goal.target_pose.pose.orientation.x = q[0]
    expected_goal.target_pose.pose.orientation.y = q[1]
    expected_goal.target_pose.pose.orientation.z = q[2]
    expected_goal.target_pose.pose.orientation.w = q[3]

    mock_get_frame.assert_called_with("map")
    mock_action_client.send_goal.call_with_args(expected_goal)
    mock_action_client.wait_for_result.call_with_args(mock_duraiton_cls(3.0))

    mobile_base.go_rel(0, 1, 2, timeout=3.0)
    mock_get_frame.assert_called_with("base")

    warnings.simplefilter("always")
    with warnings.catch_warnings(record=True) as w:
        mobile_base.go(0, 1, 2, timeout=3.0, relative=False)
        mock_get_frame.assert_called_with("map")
        eq_(w[0].category, hsrb_interface.exceptions.DeprecationWarning)

    with warnings.catch_warnings(record=True) as w:
        mobile_base.go(0, 1, 2, timeout=3.0, relative=True)
        mock_get_frame.assert_called_with("base")
        eq_(w[0].category, hsrb_interface.exceptions.DeprecationWarning)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rospy.Duration")
@patch("rospy.Time")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_pos_ori(mock_get_entry, mock_connecting,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):
    """Test MobileBase.go_pose"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    mock_action_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED

    pose = ((0, 1, 2), (0.5, 0.5, 0.5, 0.5))
    mobile_base.go_pose(pose, timeout=3.0, ref_frame_id="map")

    expected_goal = MoveBaseGoal()
    expected_goal.target_pose.header.frame_id = "map"
    expected_goal.target_pose.header.stamp = mock_time_cls(0)
    expected_goal.target_pose.pose.position.x = 0
    expected_goal.target_pose.pose.position.y = 1
    expected_goal.target_pose.pose.position.z = 2
    expected_goal.target_pose.pose.orientation.x = 0.5
    expected_goal.target_pose.pose.orientation.y = 0.5
    expected_goal.target_pose.pose.orientation.z = 0.5
    expected_goal.target_pose.pose.orientation.w = 0.5

    mock_action_client.send_goal.call_with_args(expected_goal)
    mock_action_client.wait_for_result.call_with_args(mock_duraiton_cls(3.0))

    warnings.simplefilter("always")
    with warnings.catch_warnings(record=True) as w:
        mobile_base.move(pose, timeout=3.0, ref_frame_id="map")
        eq_(w[0].category, hsrb_interface.exceptions.DeprecationWarning)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_get_pose(mock_get_entry, mock_connecting,
                              mock_action_client_cls,
                              mock_trajectory_controller):
    """Test MobileBase.get_pose()"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    ok_(mobile_base)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_go_failure(mock_get_entry,
                                mock_connecting,
                                mock_action_client_cls,
                                mock_trajectory_controller):
    """Test MobileBase.go faile if timeout is invalid"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    assert_raises(ValueError, mobile_base.go_abs, 0, 0, 0, -1)
    assert_raises(ValueError, mobile_base.go_abs, 0, 0, 0, float("inf"))
    assert_raises(ValueError, mobile_base.go_abs, 0, 0, 0, float("nan"))

    assert_raises(ValueError, mobile_base.go_rel, 0, 0, 0, -1)
    assert_raises(ValueError, mobile_base.go_rel, 0, 0, 0, float("inf"))
    assert_raises(ValueError, mobile_base.go_rel, 0, 0, 0, float("nan"))


@patch("hsrb_interface.trajectory.wait_controllers")
@patch("hsrb_interface.trajectory.timeopt_filter")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_follow_trajectory(mock_get_entry,
                                       mock_connecting,
                                       mock_action_client_cls,
                                       mock_trajectory_controller,
                                       mock_get_frame,
                                       mock_transform_trajectory,
                                       mock_timeopt_filter,
                                       mock_wait_controllers):
    """Test MobileBase.follow with poses"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"
    mock_transform_trajectory.return_value = "piyo"
    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_timeopt_filter.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    mobile_base.follow_trajectory(poses)

    mock_get_frame.assert_called_with("map")
    mock_timeopt_filter.assert_called_with("piyo")
    mock_follow_client = mock_trajectory_controller.return_value
    mock_follow_client.submit.assert_called_with(trajectory)
    mock_wait_controllers.assert_called_with([mock_follow_client])

    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "hoge")
    eq_(len(trajectory.points), 3)
    assert_almost_equal(trajectory.points[0].transforms[0].translation.x, 2.0)
    assert_almost_equal(trajectory.points[1].transforms[0].translation.x, 1.0)
    assert_almost_equal(trajectory.points[2].transforms[0].translation.x, 0.0)

    # Set ref_frame_id
    mock_get_frame.reset_mock()
    mobile_base.follow_trajectory(poses, ref_frame_id="var")

    mock_get_frame.assert_not_called()
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "var")


@patch("hsrb_interface.trajectory.wait_controllers")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_follow_trajectory_with_stamp(mock_get_entry,
                                                  mock_connecting,
                                                  mock_action_client_cls,
                                                  mock_trajectory_controller,
                                                  mock_get_frame,
                                                  mock_transform_trajectory,
                                                  mock_wait_controllers):
    """Test MobileBase.follow with stamped poses"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"

    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_transform_trajectory.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    mobile_base.follow_trajectory(poses, [3.0, 6.0])

    mock_get_frame.assert_called_with("map")
    mock_follow_client = mock_trajectory_controller.return_value
    mock_wait_controllers.assert_called_with([mock_follow_client])

    submitted_trajectory = mock_follow_client.submit.call_args[0][0]
    eq_(len(submitted_trajectory.points), 2)
    assert_almost_equal(
        submitted_trajectory.points[0].time_from_start.to_sec(), 3.0)
    assert_almost_equal(
        submitted_trajectory.points[1].time_from_start.to_sec(), 6.0)

    # Length of time_from_starts and poses should be same
    assert_raises(ValueError, mobile_base.follow_trajectory,
                  poses, [3.0])
    assert_raises(ValueError, mobile_base.follow_trajectory,
                  poses, [0.0, 3.0, 6.0])


@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_create_go_pose_goal(mock_get_entry,
                             mock_connecting,
                             mock_action_client_cls,
                             mock_trajectory_controller,
                             mock_get_frame):
    """Test MobileBase.create_move_goal"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    goal = mobile_base.create_go_pose_goal(hsrb_interface.geometry.pose(x=1.0))
    eq_(goal.header.frame_id, "hoge")
    assert_almost_equal(goal.pose.position.x, 1.0)

    goal = mobile_base.create_go_pose_goal(hsrb_interface.geometry.pose(),
                                           "piyo")
    eq_(goal.header.frame_id, "piyo")
    assert_almost_equal(goal.pose.position.x, 0.0)


@patch("hsrb_interface.trajectory.timeopt_filter")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_create_follow_goal(mock_get_entry,
                            mock_connecting,
                            mock_action_client_cls,
                            mock_trajectory_controller,
                            mock_get_frame,
                            mock_transform_trajectory,
                            mock_timeopt_filter):
    """Test MobileBase.create_follow_trajectory_goal"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"
    mock_transform_trajectory.return_value = "piyo"
    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_timeopt_filter.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    # Without time_from_starts
    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    goal = mobile_base.create_follow_trajectory_goal(poses)

    mock_get_frame.assert_called_with("map")
    mock_timeopt_filter.assert_called_with("piyo")
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "hoge")
    eq_(len(trajectory.points), 3)
    assert_almost_equal(trajectory.points[0].transforms[0].translation.x, 2.0)
    assert_almost_equal(trajectory.points[1].transforms[0].translation.x, 1.0)
    assert_almost_equal(trajectory.points[2].transforms[0].translation.x, 0.0)

    # Set ref_frame_id
    mock_get_frame.reset_mock()
    goal = mobile_base.create_follow_trajectory_goal(poses, ref_frame_id="var")

    mock_get_frame.assert_not_called()
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "var")

    # With time_from_starts
    mock_get_frame.reset_mock()
    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_transform_trajectory.return_value = trajectory
    goal = mobile_base.create_follow_trajectory_goal(poses, [3.0, 6.0])

    mock_get_frame.assert_called_with("map")
    eq_(len(goal.points), 2)
    assert_almost_equal(goal.points[0].time_from_start.to_sec(), 3.0)
    assert_almost_equal(goal.points[1].time_from_start.to_sec(), 6.0)

    # Length of time_from_starts and poses should be same
    assert_raises(ValueError, mobile_base.create_follow_trajectory_goal,
                  poses, [3.0])
    assert_raises(ValueError, mobile_base.create_follow_trajectory_goal,
                  poses, [0.0, 3.0, 6.0])


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_execute(mock_get_entry,
                 mock_connecting,
                 mock_action_client_cls,
                 mock_trajectory_controller):
    """Test MobileBase.execute"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    input_goal = PoseStamped()
    mobile_base.execute(input_goal)
    action_goal = mock_action_client.send_goal.call_args[0][0]
    eq_(action_goal.target_pose, input_goal)

    mobile_base.execute(JointTrajectory())
    mock_follow_client.submit.assert_called_with(JointTrajectory())

    assert_raises(ValueError, mobile_base.execute, "hoge")


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_is_moving(mock_get_entry,
                   mock_connecting,
                   mock_action_client_cls,
                   mock_trajectory_controller):
    """Test MobileBase.is_moving"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    # mobile_base.execute is not called
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    assert_false(mobile_base.is_moving())

    # Send pose
    mobile_base.execute(PoseStamped())
    mock_action_client.get_state.return_value = actionlib.GoalStatus.ACTIVE
    assert_true(mobile_base.is_moving())

    mock_action_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED
    assert_false(mobile_base.is_moving())

    # Send trajetory
    mobile_base.execute(JointTrajectory())
    mock_follow_client.get_state.return_value = actionlib.GoalStatus.ACTIVE
    assert_true(mobile_base.is_moving())

    mock_follow_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED
    assert_false(mobile_base.is_moving())


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_is_succeeded(mock_get_entry,
                      mock_connecting,
                      mock_action_client_cls,
                      mock_trajectory_controller):
    """Test MobileBase.is_moving"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    # mobile_base.execute is not called
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    assert_false(mobile_base.is_succeeded())

    # Send pose
    mobile_base.execute(PoseStamped())
    mock_action_client.get_state.return_value = actionlib.GoalStatus.ACTIVE
    assert_false(mobile_base.is_succeeded())

    mock_action_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED
    assert_true(mobile_base.is_succeeded())

    # Send trajetory
    mobile_base.execute(JointTrajectory())
    mock_follow_client.get_state.return_value = actionlib.GoalStatus.ACTIVE
    assert_false(mobile_base.is_succeeded())

    mock_follow_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED
    assert_true(mobile_base.is_succeeded())


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_cancel_goal(mock_get_entry,
                     mock_connecting,
                     mock_action_client_cls,
                     mock_trajectory_controller):
    """Test MobileBase.cancel_goal"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    # Cancel without goal
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mobile_base.cancel_goal()

    mock_action_client.cancel_goal.assert_not_called()
    mock_follow_client.cancel.assert_not_called()

    # Send pose and cancel
    mobile_base.execute(PoseStamped())
    mock_action_client.get_state.return_value = actionlib.GoalStatus.ACTIVE
    mobile_base.cancel_goal()

    mock_action_client.cancel_goal.assert_called_once_with()
    mock_follow_client.cancel.assert_not_called()

    # Send trajectory and cancel
    mobile_base.execute(JointTrajectory())
    mock_follow_client.get_state.return_value = actionlib.GoalStatus.ACTIVE
    mobile_base.cancel_goal()

    mock_follow_client.cancel.assert_called_once_with()
