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

"""Unittest for hsrb_interface_py.joint_group module"""
from __future__ import absolute_import

import math
import os
import sys
from unittest.mock import ANY, call, MagicMock, patch, PropertyMock

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped

from hsrb_interface_py import _testing as testing
from hsrb_interface_py import geometry
from hsrb_interface_py import joint_group
from hsrb_interface_py import robot_model
from hsrb_interface_py import settings

from hsrb_interface_py.joint_group import JointGroup

from moveit_msgs.msg import MoveItErrorCodes

from nose.tools import assert_almost_equal
# from nose.tools import assert_false
from nose.tools import assert_in
from nose.tools import assert_not_in
from nose.tools import assert_true
from nose.tools import eq_
from nose.tools import raises

from rclpy.duration import Duration
from sensor_msgs.msg import JointState
import std_msgs.msg

from tmc_planning_msgs.srv import PlanWithHandGoals
from tmc_planning_msgs.srv import PlanWithHandLine
from tmc_planning_msgs.srv import PlanWithJointGoals
from tmc_planning_msgs.srv import PlanWithTsrConstraints

import xacro


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class WholeBodyTest(testing.RosMockTestCase):

    def setUp(self):
        super(WholeBodyTest, self).setUp()

        patcher = patch("threading.Event.wait")
        self.event_wait_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface_py.trajectory.TrajectoryController")
        self.traj_controller_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("trajectory_msgs.msg.JointTrajectory")
        self.joint_trajectory_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # patcher = patch("hsrb_interface_py.trajectory.ImpedanceController")
        # self.imp_controller_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface_py.trajectory.wait_controllers")
        self.wait_controllers_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface_py.robot._get_tf2_buffer")
        self.get_tf2_buffer_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.tf2_buffer_mock = self.get_tf2_buffer_mock.return_value

        patcher = patch("hsrb_interface_py.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)
        caching_sub_mock = self.caching_sub_mock.return_value
        data_mock = PropertyMock(return_value=self.joint_state_fixture())
        # PropertyMock must be attached to class
        type(caching_sub_mock).data = data_mock

        patcher = patch(
            "hsrb_interface_py.joint_group.JointGroup._constrain_trajectories")
        self.constrain_trajectories_mock = patcher.start()
        self.addCleanup(patcher.stop)

        def get_frame_side_effect(key):
            mapping = {
                "map": {
                    "frame_id": "map"
                },
                "odom": {
                    "frame_id": "odom"
                },
                "base": {
                    "frame_id": "base_footprint"
                },
                "hand": {
                    "frame_id": "hand_palm_link"
                }
            }
            return mapping[key]["frame_id"]
        self.get_frame_mock.side_effect = get_frame_side_effect

        self.joint_group_setting = {
            "class": ["joint_group", "JointGroup"],
            "joint_states_topic": "/hsrb/joint_states",
            "arm_controller_prefix": "/hsrb/arm_trajectory_controller",
            "head_controller_prefix": "/hsrb/head_trajectory_controller",
            "hand_controller_prefix": "/hsrb/gripper_controller",
            "omni_base_controller_prefix": "/hsrb/omni_base_controller",
            "plan_with_constraints_service": "/plan_with_constraints",
            "plan_with_hand_goals_service": "/plan_with_hand_goals",
            "plan_with_hand_line_service": "/plan_with_hand_line",
            "plan_with_joint_goals_service": "/plan_with_joint_goals",
            "timeout": 1.0,
            "end_effector_frames": [
                "hand_palm_link",
                "hand_l_finger_vacuum_frame"
            ],
            "rgbd_sensor_frame": "rgbd_sensor_frame",
            "passive_joints": [
                "hand_r_spring_proximal_joint",
                "hand_l_spring_proximal_joint"
            ],
            "looking_hand_constraint": {
                "plugin_name": "LookHand",
                "use_joints": ["head_joint"]
            },
            "motion_planning_joints": [
                "wrist_flex_joint",
                "wrist_roll_joint",
                "arm_roll_joint",
                "arm_flex_joint",
                "arm_lift_joint"
            ]
        }
        self.trajectory_setting = {
            "impedance_control": "/hsrb/impedance_control",
            "constraints_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "filter_timeout": 30.0,
            "action_timeout": 30.0,
            "watch_rate": 30.0
        }

    def joint_state_fixture(self):
        joint_state = JointState()
        joint_state.header.stamp.sec = 1755
        joint_state.header.stamp.nanosec = 742000000
        joint_state.header.frame_id = ''
        joint_state.name = ['arm_flex_joint',
                            'arm_lift_joint',
                            'arm_roll_joint',
                            'base_l_drive_wheel_joint',
                            'base_r_drive_wheel_joint',
                            'base_roll_joint',
                            'hand_l_spring_proximal_joint',
                            'hand_motor_joint',
                            'hand_r_spring_proximal_joint',
                            'head_pan_joint',
                            'head_tilt_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint']
        joint_state.position = [-1.205073719123817e-05,
                                -2.353910232189444e-06,
                                6.731640830537344e-06,
                                0.00015132648675386662,
                                0.06711735857956125,
                                -1.6177818213947148e-06,
                                5.231850970766061e-05,
                                -4.0684104896548945e-05,
                                -7.597231270750626e-06,
                                5.737535211380873e-07,
                                0.0010059283317085388,
                                -0.0003140775579737465,
                                -1.972350262580136e-05]
        joint_state.velocity = [-4.398963494311168e-05,
                                0.0008855797998572822,
                                0.005013974899631351,
                                0.005780376305309976,
                                -0.0006153048272357485,
                                -0.0009519281689300109,
                                0.010811523126418292,
                                0.0015249207867800074,
                                0.0006157871810664606,
                                -0.001243430418769777,
                                0.0009314696615672182,
                                -0.0018553772677545174,
                                -0.005630832679255584]
        joint_state.effort = [0.537512390465002,
                              1.2225493661258626,
                              0.02403405582857048,
                              0.15090609010962908,
                              -0.06012715388461867,
                              -0.35157021739766886,
                              0.0,
                              0.0,
                              0.0,
                              -0.0026258098408504793,
                              -1.2449754755678555,
                              0.2814905678629742,
                              -0.017750577272845902]
        return joint_state

    def initial_tf_fixtures(self):
        odom_to_robot_transform = TransformStamped()
        odom_to_robot_transform.header.stamp.sec = 130
        odom_to_robot_transform.header.stamp.nanosec = 422000000
        odom_to_robot_transform.header.frame_id = 'odom'
        odom_to_robot_transform.child_frame_id = 'base_footprint'
        odom_to_robot_transform.transform.translation.x = 0.0
        odom_to_robot_transform.transform.translation.y = 0.0
        odom_to_robot_transform.transform.translation.z = 0.0
        odom_to_robot_transform.transform.rotation.x = 0.0
        odom_to_robot_transform.transform.rotation.y = 0.0
        odom_to_robot_transform.transform.rotation.z = 0.0
        odom_to_robot_transform.transform.rotation.w = 1.0

        odom_to_hand_transform = TransformStamped()
        odom_to_hand_transform.header.stamp.sec = 117
        odom_to_hand_transform.header.stamp.nanosec = 900000000
        odom_to_hand_transform.header.frame_id = 'odom'
        odom_to_hand_transform.child_frame_id = 'hand_palm_link'
        odom_to_hand_transform.transform.translation.x = 0.158027351797
        odom_to_hand_transform.transform.translation.y = 0.0785128775482
        odom_to_hand_transform.transform.translation.z = 0.825494221876
        odom_to_hand_transform.transform.rotation.x = 0.000202774340021
        odom_to_hand_transform.transform.rotation.y = 1.89798372121e-07
        odom_to_hand_transform.transform.rotation.z = 0.999999526031
        odom_to_hand_transform.transform.rotation.w = -0.000952270991235

        return odom_to_robot_transform, odom_to_hand_transform

    def test_creation(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        JointGroup('whole_body')
        self.get_entry_mock.assert_has_calls([
            call('joint_group', 'whole_body'),
            # call('trajectory', 'impedance_control'),
        ])
        self.traj_controller_mock.assert_has_calls([
            call(self.joint_group_setting['arm_controller_prefix'],
                 self.connection_mock),
            call(self.joint_group_setting['head_controller_prefix'],
                 self.connection_mock),
            call(self.joint_group_setting['hand_controller_prefix'],
                 self.connection_mock),
            call(self.joint_group_setting['omni_base_controller_prefix'],
                 self.connection_mock,
                 'base_coordinates'),
        ])
        # self.imp_controller_mock.assert_called_with(self.trajectory_setting)

    def test_end_effector_frame_ok(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]

        whole_body = JointGroup('whole_body')
        eq_('hand_palm_link', whole_body.end_effector_frame)
        frames = ('hand_palm_link', 'hand_l_finger_vacuum_frame')
        eq_(frames, whole_body.end_effector_frames)
        whole_body.end_effector_frame = 'hand_l_finger_vacuum_frame'
        eq_('hand_l_finger_vacuum_frame', whole_body.end_effector_frame)

    @raises(ValueError)
    def test_end_effector_frame_ng(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.end_effector_frame = 'hoge'

    def test_joint_state(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        eq_(self.joint_state_fixture(), whole_body.joint_state)

    def test_get_robot_urdf(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body._get_robot_urdf()
        self.connection_mock.create_subscription.assert_called_with(
            std_msgs.msg.String,
            'robot_description',
            ANY,
            ANY)

    def test_joint_limits(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        xacro_file = xacro.process_file(
            os.path.join(
                get_package_share_directory('hsrb_description'),
                'robots',
                'hsrb4s.urdf.xacro'
            )
        )
        urdf_xml = xacro_file.toprettyxml(indent='  ')
        model = robot_model.RobotModel.from_xml_string(urdf_xml)

        whole_body = JointGroup('whole_body')
        expected = {
            'arm_flex_joint': (-2.62, 0.0),
            'arm_lift_joint': (0.0, 0.69),
            'arm_roll_joint': (-2.09, 3.84),
            'base_l_drive_wheel_joint': (0, 0),
            'base_r_drive_wheel_joint': (0, 0),
            'base_roll_joint': (0, 0),
            'hand_l_spring_proximal_joint': (0.0, 0.698),
            'hand_motor_joint': (-0.798, 1.24),
            'hand_r_spring_proximal_joint': (0.0, 0.698),
            'head_pan_joint': (-3.84, 1.75),
            'head_tilt_joint': (-1.57, 0.52),
            'wrist_flex_joint': (-1.92, 1.22),
            'wrist_roll_joint': (-1.92, 3.67),
        }
        whole_body._robot_urdf = model
        eq_(whole_body.joint_limits, expected)

    @raises(ValueError)
    def test_planning_timeout_non_value(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.planning_timeout = "hoge"

    @raises(ValueError)
    def test_planning_timeout_negative(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.planning_timeout = -1.0

    @raises(ValueError)
    def test_linear_weight_non_value(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.linear_weight = "hoge"

    @raises(ValueError)
    def test_linear_weight_negative(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.linear_weight = -1.0

    @raises(ValueError)
    def test_angular_weight_non_value(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.angular_weight = "hoge"

    @raises(ValueError)
    def test_angular_weight_negative(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.angular_weight = -1.0

    @raises(ValueError)
    def test_joint_weights_non_dict(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.joint_weights = [1, 2, 3]

    @raises(ValueError)
    def test_joint_weights_negative(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.joint_weights = {"arm_flex_joint": -3}

    @raises(ValueError)
    def test_joint_weights_zero(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.joint_weights = {"arm_flex_joint": 0.0}

    @raises(ValueError)
    def test_joint_weights_non_in_joint_list(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.joint_weights = {"arm_pitch_joint": 3}

    def test_joint_weights_ok(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        plan_service_client_mock = self.connection_mock.create_client()
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(
            return_value=MoveItErrorCodes.SUCCESS)
        type(plan_result_mock.error_code).val = error_code_mock
        plan_service_client_mock.call_async().result.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.joint_weights = {"arm_flex_joint": 30.0}
        whole_body.move_end_effector_by_line(axis=(0, 0, 1), distance=1.0)

        # Check planning request
        request = plan_service_client_mock.call_async.call_args[0][0]
        assert_in('arm_flex_joint', request.weighted_joints)
        eq_(request.weight[request.weighted_joints.index(
            'arm_flex_joint')], 30.0)

        whole_body.joint_weights = {"arm_lift_joint": 42.0}
        whole_body.move_end_effector_by_line(axis=(0, 0, 1), distance=1.0)

        # Check to reset request.weighted_joints
        request = plan_service_client_mock.call_async.call_args[0][0]
        assert_not_in('arm_flex_joint', request.weighted_joints)
        assert_in('arm_lift_joint', request.weighted_joints)

    def test_get_end_effector_pose(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]

        whole_body = JointGroup('whole_body')
        whole_body.get_end_effector_pose()

        # Check post-conditions
        self.tf2_buffer_mock.lookup_transform.assert_called_with(
            settings.get_frame('base'),
            settings.get_frame('hand'),
            self.connection_mock.get_clock().now(),
            Duration(seconds=5.0)
        )

    def test_move_to_joint_positions_ok(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        plan_service_client_mock = self.connection_mock.create_client()
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(
            return_value=MoveItErrorCodes.SUCCESS)
        type(plan_result_mock.error_code).val = error_code_mock
        plan_service_client_mock.call_async().result.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.5})

        # Check post-conditions
        service = self.joint_group_setting["plan_with_joint_goals_service"]
        self.connection_mock.create_client.assert_called_with(
            PlanWithJointGoals,
            service)
        plan_service_client_mock.call_async.assert_called_with(ANY)

    @raises(ValueError)
    def test_move_to_base_roll_joint(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions({'base_roll_joint': 1.0})

    @raises(ValueError)
    def test_move_to_passive_joint(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions(
            {'hand_r_spring_proximal_joint': 1.0})

    def test_move_end_effector_pose_single(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]

        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        plan_service_client_mock = self.connection_mock.create_client()
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(
            return_value=MoveItErrorCodes.SUCCESS)
        type(plan_result_mock.error_code).val = error_code_mock
        plan_service_client_mock.call_async().result.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_end_effector_pose(geometry.pose(1, 0, 1))

        # Check post-conditions
        service = self.joint_group_setting["plan_with_hand_goals_service"]
        self.connection_mock.create_client.assert_called_with(
            PlanWithHandGoals,
            service)
        plan_service_client_mock.call_async.assert_called_with(ANY)

        request = plan_service_client_mock.call_async.call_args[0][0]
        eq_(len(request.origin_to_hand_goals), 1)

    def test_move_end_effector_pose_multi(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]

        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        plan_service_client_mock = self.connection_mock.create_client()
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(
            return_value=MoveItErrorCodes.SUCCESS)
        type(plan_result_mock.error_code).val = error_code_mock
        plan_service_client_mock.call_async().result.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_end_effector_pose([geometry.pose(1, 0, 1),
                                           geometry.pose(2, 1, 0)])

        # Check post-conditions
        service = self.joint_group_setting["plan_with_hand_goals_service"]
        self.connection_mock.create_client.assert_called_with(
            PlanWithHandGoals,
            service)
        plan_service_client_mock.call_async.assert_called_with(ANY)

        request = plan_service_client_mock.call_async.call_args[0][0]
        eq_(len(request.origin_to_hand_goals), 2)

    def test_move_end_effector_by_line_ok(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.return_value = odom_to_robot_transform
        plan_service_client_mock = self.connection_mock.create_client()
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(
            return_value=MoveItErrorCodes.SUCCESS)
        type(plan_result_mock.error_code).val = error_code_mock
        plan_service_client_mock.call_async().result.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_end_effector_by_line(axis=(0, 0, 1), distance=1.0)

        # Check post-conditions
        self.tf2_buffer_mock.lookup_transform.assert_called_with(
            settings.get_frame('odom'),
            settings.get_frame('base'),
            self.connection_mock.get_clock().now(),
            Duration(seconds=5.0)
        )
        service = self.joint_group_setting["plan_with_hand_line_service"]
        self.connection_mock.create_client.assert_called_with(
            PlanWithHandLine,
            service)
        plan_service_client_mock.call_async.assert_called_with(ANY)

    @raises(ValueError)
    def test_move_end_effector_by_line_ng(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.return_value = odom_to_robot_transform
        whole_body = JointGroup('whole_body')
        whole_body.move_end_effector_by_line(axis=(0, 0, 0), distance=0.1)

    def test_move_end_effector_by_arc_ok(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = \
            self.initial_tf_fixtures()
        self.tf2_buffer_mock.lookup_transform.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
            odom_to_robot_transform,
        ]
        plan_service_client_mock = self.connection_mock.create_client()
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(
            return_value=MoveItErrorCodes.SUCCESS)
        type(plan_result_mock.error_code).val = error_code_mock
        plan_service_client_mock.call_async().result.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_end_effector_by_arc(geometry.pose(), 1.0)

        # Check post-conditions
        service = self.joint_group_setting["plan_with_constraints_service"]
        self.connection_mock.create_client.assert_called_with(
            PlanWithTsrConstraints,
            service)
        plan_service_client_mock.call_async.assert_called_with(ANY)

    @raises(ValueError)
    def test_move_end_effector_by_arc_ng(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_end_effector_by_arc(geometry.pose(), 10.0)

    def test_inverse_pose(self):
        pose = geometry.pose(1, 2, 3)
        inv = joint_group._invert_pose(pose)
        assert_true(isinstance(inv.pos, geometry.Vector3))
        assert_true(isinstance(inv.ori, geometry.Quaternion))
        result = geometry.multiply_tuples(inv, pose)
        assert_almost_equal(result[0][0], 0)
        assert_almost_equal(result[0][1], 0)
        assert_almost_equal(result[0][2], 0)
        assert_almost_equal(result[1][0], 0)
        assert_almost_equal(result[1][1], 0)
        assert_almost_equal(result[1][2], 0)
        assert_almost_equal(result[1][3], 1)

        pose = geometry.pose(ej=math.pi / 4.0)
        inv = joint_group._invert_pose(pose)
        result = geometry.multiply_tuples(inv, pose)
        assert_almost_equal(result[0][0], 0)
        assert_almost_equal(result[0][1], 0)
        assert_almost_equal(result[0][2], 0)
        assert_almost_equal(result[1][0], 0)
        assert_almost_equal(result[1][1], 0)
        assert_almost_equal(result[1][2], 0)
        assert_almost_equal(result[1][3], 1)

        pose = geometry.pose(1, 3, 5,
                             ei=math.pi / 3.0,
                             ej=math.pi / 4.0,
                             ek=-math.pi / 8)
        inv = joint_group._invert_pose(pose)
        result = geometry.multiply_tuples(inv, pose)
        assert_almost_equal(result[0][0], 0)
        assert_almost_equal(result[0][1], 0)
        assert_almost_equal(result[0][2], 0)
        assert_almost_equal(result[1][0], 0)
        assert_almost_equal(result[1][1], 0)
        assert_almost_equal(result[1][2], 0)
        assert_almost_equal(result[1][3], 1)

    def test_movement_axis_and_distance(self):
        pose1 = geometry.pose(0, 0, 0)
        pose2 = geometry.pose(2, 0, 0)
        axis, distance = joint_group._movement_axis_and_distance(pose1, pose2)
        assert_almost_equal(axis[0], 1)
        assert_almost_equal(axis[1], 0)
        assert_almost_equal(axis[2], 0)
        assert_almost_equal(distance, 2)
