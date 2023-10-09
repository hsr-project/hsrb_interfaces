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

"""Unittest for end_effector module"""
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import hsrb_interface
from hsrb_interface import _testing as testing
import hsrb_interface.end_effector
import hsrb_interface.exceptions

from mock import ANY
from nose.tools import assert_raises
from nose.tools import ok_
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal
from trajectory_msgs.msg import JointTrajectoryPoint

_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_HAND_MOMENT_ARM_LENGTH = 0.07


class EndEffectorTest(testing.RosMockTestCase):

    def setUp(self):
        super(EndEffectorTest, self).setUp()
        self.gripper_setting = {
            'topic': 'hoge',
            "joint_names": ["hand_motor_joint"],
            "prefix": "/hsrb/gripper_controller",
            "left_finger_joint_name": "hand_l_spring_proximal_joint",
            "right_finger_joint_name": "hand_r_spring_proximal_joint"
        }
        self.suction_setting = {
            "action": "/suction_control",
            "suction_topic": "/suction_on",
            "pressure_sensor_topic": "/pressure_sensor"
        }

    def test_gripper(self):
        """Test Gripper class"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        gripper = hsrb_interface.end_effector.Gripper('foo')
        ok_(gripper)
        self.action_client_mock.assert_any_call(
            "/hsrb/gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        self.action_client_mock.assert_any_call(
            "/hsrb/gripper_controller/grasp",
            GripperApplyEffortAction)
        self.action_client_mock.assert_any_call(
            "/hsrb/gripper_controller/apply_force",
            GripperApplyEffortAction)
        self.subscriber_mock.assert_called_with(
            "/hsrb/joint_states",
            JointState,
            callback=ANY)

    def test_gripper_command(self):
        """Test command"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        open_angle = 1.2
        gripper.command(open_angle)
        expected_goal = FollowJointTrajectoryGoal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rospy.Duration(1.0))
        ]
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_command_set_motion_time(self):
        """Test command when set motion_time"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        open_angle = 1.2
        motion_time = 2.0
        gripper.command(open_angle, motion_time=motion_time)
        expected_goal = FollowJointTrajectoryGoal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=rospy.Duration(motion_time)
            )
        ]
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_command_sync_false(self):
        """Test command when sync are false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface.end_effector.Gripper('foo')
        open_angle = 1.2
        gripper.command(open_angle, sync=False)
        self.assertFalse(action_client_mock.wait_for_result.called)
        expected_goal = FollowJointTrajectoryGoal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rospy.Duration(1.0))
        ]
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_false(self):
        """Test apply force when delicate is false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = 1.0
        gripper.apply_force(effort)
        expected_goal = GripperApplyEffortGoal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_false_sync_false(self):
        """Test apply force when delicate and sync are false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = 1.0
        gripper.apply_force(effort, sync=False)
        self.assertFalse(action_client_mock.wait_for_result.called)
        expected_goal = GripperApplyEffortGoal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_true_but_equal_to_threshold(self):
        """Test apply force when delicate is true but equal to threshold"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD
        gripper.apply_force(effort, delicate=True)
        expected_goal = GripperApplyEffortGoal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_true_and_less_than_threshold(self):
        """Test apply force when delicate is true and less than threshold"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD - 0.01
        gripper.apply_force(effort, delicate=True)
        expected_goal = GripperApplyEffortGoal()
        expected_goal.effort = effort
        action_client_mock.send_goal.assert_called_with(expected_goal)

    def test_gripper_apply_force_effort_negative_value(self):
        """Test apply force(negative effort is set)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, -1.0)

    def test_gripper_apply_force_fail(self):
        """Test apply force(failed to apply force)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.PREEMPTED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, 1.0)

    def test_gripper_apply_force_time_out(self):
        """Test apply force(timed out)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = False
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, 1.0)

    def test_suction(self):
        """Test Suction class"""
        self.get_entry_mock.side_effect = [self.suction_setting]
        suction = hsrb_interface.end_effector.Suction('foo')
        ok_(suction)
        self.get_entry_mock.assert_called_with('end_effector', 'foo')

    def test_suction_command(self):
        """Test Suction command(exceptions are never thrown)"""
        self.get_entry_mock.side_effect = [self.suction_setting]
        suction = hsrb_interface.end_effector.Suction('foo')
        suction.command(True)
        suction.command(False)
        self.publisher_mock.assert_called_with("/suction_on",
                                               Bool,
                                               queue_size=0)

    def test_suction_command_negative_num(self):
        """Test Suction command(negative number)"""
        self.get_entry_mock.side_effect = [self.suction_setting]
        suction = hsrb_interface.end_effector.Suction('foo')
        assert_raises(ValueError, suction.command, -1)
