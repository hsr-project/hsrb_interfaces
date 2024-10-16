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
"""Unittest for hsrb_interface.joint_group module"""

import math
import os
import sys

from unittest.mock import ANY
from unittest.mock import MagicMock
from unittest.mock import patch
from unittest.mock import PropertyMock

import _testing as testing
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import TransformStamped
from hsrb_interface import geometry
from hsrb_interface.joint_group import JointGroup
from hsrb_interface.robot import Robot
from moveit_msgs.msg import MoveItErrorCodes

from nose.tools import eq_
from nose.tools import raises

from sensor_msgs.msg import JointState
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_planning_msgs.srv import PlanWithJointGoals
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_MANIPULATION_AREA_SIZE = 100.0


class WholeBodyTest(testing.RosMockTestCase):

    def setUp(self):
        super(WholeBodyTest, self).setUp()

        patcher = patch("hsrb_interface.trajectory.TrajectoryController")
        self.traj_controller_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.trajectory.wait_controllers")
        self.wait_controllers_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.robot._get_tf2_buffer")
        self.get_tf2_buffer_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.tf2_buffer_mock = self.get_tf2_buffer_mock.return_value

        description_package_path = get_package_share_path('hsrb_description')
        urdf_xml = os.path.join(description_package_path, 'robots', 'hsrb4s.urdf.xacro')
        with open(urdf_xml) as f:
            model = [f.read()]
        self.get_param_mock.return_value = model

        patcher = patch("hsrb_interface.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)
        caching_sub_mock = self.caching_sub_mock.return_value
        data_mock = PropertyMock(return_value=self.joint_state_fixture())
        # PropertyMock must be attached to class
        type(caching_sub_mock).data = data_mock

        patcher = patch("hsrb_interface.joint_group.KinematicsInterface")
        self.kinematics_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("rclpy.spin_until_future_complete")
        self.spin_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("asyncio.run")
        self.async_run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch.object(Robot, "_connection")
        self.joint_group_node_mock_obj = patcher.start()
        self.addCleanup(patcher.stop)
        self.create_client_mock = self.joint_group_node_mock_obj.create_client

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
            "joint_states_topic": "/joint_states",
            "arm_controller_prefix": "/arm_trajectory_controller",
            "head_controller_prefix": "/head_trajectory_controller",
            "hand_controller_prefix": "/gripper_controller",
            "omni_base_controller_prefix": "/omni_base_controller",
            "plan_with_constraints_service": "/plan_with_constraints",
            "plan_with_hand_goals_service": "/plan_with_hand_goals",
            "plan_with_hand_line_service": "/plan_with_hand_line",
            "plan_with_joint_goals_service": "/plan_with_joint_goals",
            "timeout": 30.0,
            "end_effector_frames": [
                "hand_palm_link",
                "hand_l_finger_vacuum_frame"
            ],
            "rgbd_sensor_frame": "head_rgbd_sensor_link",
            "passive_joints": [
                "hand_r_spring_proximal_joint",
                "hand_l_spring_proximal_joint"
            ],
            "looking_hand_constraint": {
                "plugin_name": "hsrb_planner_plugins/LookHand",
                "use_joints": ["head_pan_joint", "head_tilt_joint"]
            },
            "motion_planning_joints": [
                "wrist_flex_joint",
                "wrist_roll_joint",
                "arm_roll_joint",
                "arm_flex_joint",
                "arm_lift_joint",
                "hand_motor_joint",
                "head_pan_joint",
                "head_tilt_joint"
            ]
        }

        self.trajectory_setting = {
            "impedance_control": "/hsrb/impedance_control",
            "constraint_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "whole_timeopt_filter_service": "/timeopt_filter_node/filter_trajectory",
            "caster_joint": "base_roll_joint",
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

    def test_constraint_tsrs(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = self.initial_tf_fixtures()
        self.async_run_mock.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        whole_body = JointGroup('whole_body')

        constraint_tsr = TaskSpaceRegion()
        constraint_tsr.end_frame_id = whole_body.end_effector_frame
        constraint_tsr.origin_to_tsr.orientation.w = 1.0
        constraint_tsr.tsr_to_end = geometry.tuples_to_pose(
            whole_body.get_end_effector_pose('odom')
        )
        constraint_tsr.min_bounds[0] = -_MANIPULATION_AREA_SIZE
        constraint_tsr.min_bounds[1] = -_MANIPULATION_AREA_SIZE
        constraint_tsr.min_bounds[2] = -_MANIPULATION_AREA_SIZE
        constraint_tsr.max_bounds[0] = _MANIPULATION_AREA_SIZE
        constraint_tsr.max_bounds[1] = _MANIPULATION_AREA_SIZE
        constraint_tsr.max_bounds[2] = _MANIPULATION_AREA_SIZE
        constraint_tsr.min_bounds[3] = -0.3
        constraint_tsr.min_bounds[4] = -0.3
        constraint_tsr.max_bounds[3] = 0.3
        constraint_tsr.max_bounds[4] = 0.3
        constraint_tsr.min_bounds[5] = -math.pi
        constraint_tsr.max_bounds[5] = math.pi

        whole_body.constraint_tsrs = [constraint_tsr]

        eq_(constraint_tsr, whole_body.constraint_tsrs[0])

    @raises(ValueError)
    def test_constraint_tsrs_not_list(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.constraint_tsrs = "hoge"

    @raises(TypeError)
    def test_constraint_tsrs_not_tsr(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.constraint_tsrs = ["hoge"]

    def test_move_to_joint_positions_ok(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting['whole_timeopt_filter_service'],
            self.trajectory_setting['caster_joint'],
            self.trajectory_setting['watch_rate'],
        ]
        odom_to_robot_transform, odom_to_hand_transform = self.initial_tf_fixtures()
        self.async_run_mock.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        plan_service_client_mock = self.create_client_mock.return_value
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(return_value=MoveItErrorCodes.SUCCESS)
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.joint_group_setting['motion_planning_joints']
        solution_mock = PropertyMock(return_value=joint_trajectory)
        type(plan_result_mock.result().error_code).val = error_code_mock
        type(plan_result_mock.result()).solution = solution_mock
        plan_service_client_mock.call_async.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.5})

        # Check post-conditions
        service = self.joint_group_setting["plan_with_joint_goals_service"]
        self.create_client_mock.assert_any_call(PlanWithJointGoals, service)
        plan_service_client_mock.call_async.assert_called_with(ANY)
        self.wait_controllers_mock.assert_called_with(ANY, ANY)

    @raises(ValueError)
    def test_move_to_base_roll_joint(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions({'base_roll_joint': 1.0})

    @raises(ValueError)
    def test_move_to_passive_joint(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions(
            {'hand_r_spring_proximal_joint': 1.0})

    def test_move_to_joint_positions_multiple_targets_ok(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting['whole_timeopt_filter_service'],
            self.trajectory_setting['caster_joint'],
            self.trajectory_setting['watch_rate'],
        ]
        odom_to_robot_transform, odom_to_hand_transform = self.initial_tf_fixtures()
        self.async_run_mock.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        plan_service_client_mock = self.create_client_mock.return_value
        plan_result_mock = MagicMock()
        error_code_mock = PropertyMock(return_value=MoveItErrorCodes.SUCCESS)
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.joint_group_setting['motion_planning_joints']
        solution_mock = PropertyMock(return_value=joint_trajectory)
        type(plan_result_mock.result().error_code).val = error_code_mock
        type(plan_result_mock.result()).solution = solution_mock
        plan_service_client_mock.call_async.return_value = plan_result_mock

        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions_multiple_targets(
            ['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint',
             'wrist_flex_joint', 'wrist_roll_joint'],
            [[0.0, 0.0, -1.57, -1.57, 0.0],
             [-0.3, 0.0, 0.0, -1.72, 1.57],
             [-0.3, 0.0, 0.0, -1.72, -1.57]]
        )

        # Check post-conditions
        service = self.joint_group_setting["plan_with_joint_goals_service"]
        self.create_client_mock.assert_any_call(PlanWithJointGoals, service)
        plan_service_client_mock.call_async.assert_called_with(ANY)
        self.wait_controllers_mock.assert_called_with(ANY, ANY)

    @raises(ValueError)
    def test_move_to_base_roll_joint_multiple_targets(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions_multiple_targets(
            ['base_roll_joint'],
            [[1.0]]
        )

    @raises(ValueError)
    def test_move_to_passive_joint_multiple_targets(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions_multiple_targets(
            ['hand_r_spring_proximal_joint'],
            [[1.0]]
        )

    @raises(ValueError)
    def test_move_to_targets_greater_than_joints(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = self.initial_tf_fixtures()
        self.async_run_mock.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions_multiple_targets(
            ['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'wrist_flex_joint'],
            [[0.0, 0.0, -1.57, -1.57, 0.0]]
        )

    @raises(ValueError)
    def test_move_to_targets_less_than_joints(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = self.initial_tf_fixtures()
        self.async_run_mock.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.move_to_joint_positions_multiple_targets(
            ['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint',
             'wrist_flex_joint', 'wrist_roll_joint'],
            [[0.0, 0.0, -1.57, -1.57]]
        )
