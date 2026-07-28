#!/usr/bin/env python
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
# vim: fileencoding=utf-8
"""Unittest for hsrb_interface_py.settings module."""

import json
import os
import unittest

import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.settings


class SettingTest(unittest.TestCase):
    HSRB_SETTINGS = """
    {
        "robot": {
            "hsrb": {
                "fullname": "HSR-B"
            }
        },
        "frame": {
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
        },
        "trajectory": {
            "impedance_control": "/hsrb/impedance_control",
            "constraint_filter_service": "/trajectory_filter/filter_trajectory_with_constraints",
            "whole_timeopt_filter_service": "/timeopt_filter_node/filter_trajectory",
            "caster_joint": "base_roll_joint",
            "filter_timeout": 30.0,
            "action_timeout": 3.0,
            "watch_rate": 30.0
        },
        "joint_group": {
            "whole_body": {
                "class": [
                    "joint_group",
                    "JointGroup"
                ],
                "joint_states_topic": "/joint_states",
                "joint_trajectory_controllers": [
                    "/arm_trajectory_controller",
                    "/head_trajectory_controller"
                ],
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
                    "use_joints": [
                        "head_pan_joint",
                        "head_tilt_joint"
                    ]
                },
                "use_joints_for_moving_end_effector": {
                    "hand_palm_link" : [
                        "wrist_flex_joint",
                        "wrist_roll_joint",
                        "arm_roll_joint",
                        "arm_flex_joint",
                        "arm_lift_joint"
                    ],
                    "hand_l_finger_vacuum_frame" : [
                        "wrist_flex_joint",
                        "wrist_roll_joint",
                        "arm_roll_joint",
                        "arm_flex_joint",
                        "arm_lift_joint"
                    ]
                },
                "neutral_joint_positions": {
                    "arm_lift_joint": 0.0,
                    "arm_flex_joint": 0.0,
                    "arm_roll_joint": 0.0,
                    "wrist_flex_joint": -1.57,
                    "wrist_roll_joint": 0.0,
                    "head_pan_joint": 0.0,
                    "head_tilt_joint": 0.0
                },
                "base_moving_joint_positions": {
                    "arm_flex_joint": 0.0,
                    "arm_lift_joint": 0.0,
                    "arm_roll_joint": -1.57,
                    "wrist_flex_joint": -1.57,
                    "wrist_roll_joint": 0.0,
                    "head_pan_joint": 0.0,
                    "head_tilt_joint": 0.0
                }
            }
        },
        "end_effector": {
            "gripper": {
                "class": [
                    "end_effector",
                    "Gripper"
                ],
                "joint_names": [
                    "hand_motor_joint"
                ],
                "prefix": "/gripper_controller"
            }
        },
        "mobile_base": {
            "omni_base": {
                "class": [
                    "mobile_base",
                    "MobileBase"
                ],
                "navigation_action": "/move_base/move",
                "follow_trajectory_action": "/omni_base_controller",
                "pose_topic": "/global_pose",
                "goal_topic": "/base_goal",
                "joint_states_topic": "/joint_states",
                "timeout": 1.0
            }
        },
        "text_to_speech": {
            "default_tts": {
                "class": [
                    "text_to_speech",
                    "TextToSpeech"
                ],
                "topic": "/talk_request"
            }
        },
        "collision_world": {
            "global_collision_world": {
                "class": [
                    "collision_world",
                    "CollisionWorld"
                ],
                "control_topic": "/collision_environment_server/collision_object",
                "environment_topic": "/collision_environment_server/environment",
                "trans_env_topic": "/collision_environment_server/transformed_environment",
                "set_frame_service": "/collision_environment_server/set_parameters",
                "attached_object": {
                    "hand_palm_link": {
                        "attaching_topic": "/attached_object_publisher/attaching_object_name",
                        "add_attaching_topic": "/attached_object_publisher/attaching_object_info",
                        "releasing_topic": "/attached_object_publisher/releasing_object_name",
                        "attached_info_topic": "/attached_object_publisher/attached_object"
                    }
                }
            }
        }
    }
    """

    # The settings are intentionally odd as they are only for checking setting.py
    TEST_ROBOT_SETTINGS = """
    {
        "robot": {
            "test": {
                "fullname": "test_robot"
            }
        },
        "frame": {
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
                "frame_id": [
                    "hand_left_link",
                    "hand_right_link"
                ]
            }
        },
        "trajectory": {
            "impedance_control": "/hsrb/impedance_control",
            "constraint_filter_service": "/trajectory_filter/filter_trajectory_with_constraints",
            "whole_timeopt_filter_service": "/timeopt_filter_node/filter_trajectory",
            "caster_joint": "base_roll_joint",
            "filter_timeout": 30.0,
            "action_timeout": 3.0,
            "watch_rate": 30.0
        },
        "joint_group": {
            "whole_body": {
                "class": [
                    "joint_group",
                    "JointGroup"
                ],
                "joint_states_topic": "/joint_states",
                "joint_trajectory_controllers": [
                    "/arm_left_trajectory_controller",
                    "/arm_right_trajectory_controller",
                    "/head_trajectory_controller"
                ],
                "omni_base_controller_prefix": "/omni_base_controller",
                "plan_with_constraints_service": "/plan_with_constraints",
                "plan_with_hand_goals_service": "/plan_with_hand_goals",
                "plan_with_hand_line_service": "/plan_with_hand_line",
                "plan_with_joint_goals_service": "/plan_with_joint_goals",
                "timeout": 30.0,
                "end_effector_frames": [
                    "hand_left_link",
                    "hand_right_link"
                ],
                "rgbd_sensor_frame": "head_rgbd_sensor_link",
                "passive_joints": [
                    "hand_r_spring_proximal_joint",
                    "hand_l_spring_proximal_joint"
                ],
                "looking_hand_constraint": {},
                "use_joints_for_moving_end_effector": {
                    "hand_left_link" : [
                        "wrist_left_flex_joint",
                        "wrist_left_roll_joint",
                        "arm_left_roll_joint",
                        "arm_left_flex_joint",
                        "arm_left_lift_joint"
                    ],
                    "hand_right_link" : [
                        "wrist_right_flex_joint",
                        "wrist_right_roll_joint",
                        "arm_right_roll_joint",
                        "arm_right_flex_joint",
                        "arm_right_lift_joint"
                    ]
                },
                "neutral_joint_positions": {
                    "arm_left_lift_joint": 0.0,
                    "arm_left_flex_joint": 0.0,
                    "arm_left_roll_joint": 0.0,
                    "wrist_left_flex_joint": 0.0,
                    "wrist_left_roll_joint": 0.0,
                    "arm_right_lift_joint": 0.0,
                    "arm_right_flex_joint": 0.0,
                    "arm_right_roll_joint": 0.0,
                    "wrist_right_flex_joint": 0.0,
                    "wrist_right_roll_joint": 0.0,
                    "head_pan_joint": 0.0,
                    "head_tilt_joint": 0.0
                },
                "base_moving_joint_positions": {
                    "arm_left_lift_joint": 0.0,
                    "arm_left_flex_joint": 0.0,
                    "arm_left_roll_joint": 0.0,
                    "wrist_left_flex_joint": 0.0,
                    "wrist_left_roll_joint": 0.0,
                    "arm_right_lift_joint": 0.0,
                    "arm_right_flex_joint": 0.0,
                    "arm_right_roll_joint": 0.0,
                    "wrist_right_flex_joint": 0.0,
                    "wrist_right_roll_joint": 0.0,
                    "head_pan_joint": 0.0,
                    "head_tilt_joint": 0.0
                }
            }
        },
        "end_effector": {
            "left_gripper": {
                "class": [
                    "end_effector",
                    "Gripper"
                ],
                "joint_names": [
                    "hand_left_motor_joint"
                ],
                "prefix": "/gripper_left_controller"
            },
            "right_gripper": {
                "class": [
                    "end_effector",
                    "Gripper"
                ],
                "joint_names": [
                    "hand_right_motor_joint"
                ],
                "prefix": "/gripper_right_controller"
            }
        },
        "mobile_base": {
            "omni_base": {
                "class": [
                    "mobile_base",
                    "MobileBase"
                ],
                "navigation_action": "/move_base/move",
                "follow_trajectory_action": "/omni_base_controller",
                "pose_topic": "/global_pose",
                "goal_topic": "/base_goal",
                "joint_states_topic": "/joint_states",
                "timeout": 1.0
            }
        },
        "text_to_speech": {
            "default_tts": {
                "class": [
                    "text_to_speech",
                    "TextToSpeech"
                ],
                "topic": "/talk_request"
            }
        },
        "collision_world": {
            "global_collision_world": {
                "class": [
                    "collision_world",
                    "CollisionWorldHSRF",
                    "hsrf_interface"
                ],
                "control_topic": "/collision_environment_server/collision_object",
                "environment_topic": "/collision_environment_server/environment",
                "trans_env_topic": "/collision_environment_server/transformed_environment",
                "set_frame_service": "/collision_environment_server/set_parameters",
                "attached_object": {
                    "hand_left_link": {
                        "attaching_topic": "/hand_left_attached_object_publisher/attaching_object_name",
                        "add_attaching_topic": "/hand_left_attached_object_publisher/attaching_object_info",
                        "releasing_topic": "/hand_left_attached_object_publisher/releasing_object_name",
                        "attached_info_topic": "/hand_left_attached_object_publisher/attached_object"
                    },
                    "hand_right_link": {
                        "attaching_topic": "/hand_right_attached_object_publisher/attaching_object_name",
                        "add_attaching_topic": "/hand_right_attached_object_publisher/attaching_object_info",
                        "releasing_topic": "/hand_right_attached_object_publisher/releasing_object_name",
                        "attached_info_topic": "/hand_right_attached_object_publisher/attached_object"
                    }
                }
            }
        }
    }
    """

    def check_setting_data(self, check_data):
        for section in check_data.keys():
            self.assertEqual(hsrb_interface.settings.get_section(section), check_data[section])

    def test_setting_normal(self):
        hsrb_interface.settings.load_settings()

        check_setting = json.loads(self.HSRB_SETTINGS)

        self.check_setting_data(check_setting)

    def test_setting_with_file(self):
        hsrb_interface.settings.load_settings(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_settings.json'))

        check_setting = json.loads(self.TEST_ROBOT_SETTINGS)

        self.check_setting_data(check_setting)

    def test_setting_item_not_found(self):
        with self.assertRaises(hsrb_interface.exceptions.ResourceNotFoundError):
            hsrb_interface.settings.load_settings()

            hsrb_interface.settings.get_entry_by_name('hoge')

    def test_setting_section_not_found(self):
        with self.assertRaises(hsrb_interface.exceptions.ResourceNotFoundError):
            hsrb_interface.settings.load_settings()

            hsrb_interface.settings.get_entry('hoge', 'whole_body')

    def test_setting_item_not_in_section(self):
        with self.assertRaises(hsrb_interface.exceptions.ResourceNotFoundError):
            hsrb_interface.settings.load_settings()

            hsrb_interface.settings.get_entry('joint_group', 'hoge')

    def test_setting_does_not_exist_setting_file(self):
        with self.assertRaises(hsrb_interface.exceptions.ResourceNotFoundError):
            hsrb_interface.settings.load_settings('hoge')
