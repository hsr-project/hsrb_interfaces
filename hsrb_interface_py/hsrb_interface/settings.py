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
"""Mapping table of ROS Graph names.

This module is intended to internal use only.
"""

import json
import os

from . import exceptions

VERSION = "1.0.0"

_HSRB_SETTINGS = """
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


def update_setting(update_settings):
    """Update a robot settings from json data.

    Args:
        update_settings (dict): Update json data.
    """
    for outer_key in update_settings.keys():
        if _SETTINGS[outer_key].keys() == update_settings[outer_key].keys():
            # If the keys are the same, update each key
            for inner_key in update_settings[outer_key].keys():
                _SETTINGS[outer_key][inner_key].update(update_settings[outer_key][inner_key])
        elif set(_SETTINGS[outer_key].keys()).isdisjoint(set(update_settings[outer_key].keys())):
            # Overwrite if the keys are different
            _SETTINGS[outer_key] = {}
            _SETTINGS[outer_key] = update_settings[outer_key]
        else:
            # Update the existing keys and add the non-existent keys
            for inner_key in update_settings[outer_key].keys():
                if inner_key in _SETTINGS[outer_key].keys():
                    _SETTINGS[outer_key][inner_key].update(update_settings[outer_key][inner_key])
                else:
                    _SETTINGS[outer_key][inner_key] = update_settings[outer_key][inner_key]


def load_settings(setting_file_path=''):
    """Load a robot settings from file.

    Args:
        setting_file_path (str): The path to configuraion file.

    Raises:
        hsrb_interface.exceptions.ResourceNotFoundError: Setting file is not found.
    """
    global _SETTINGS

    _SETTINGS = json.loads(_HSRB_SETTINGS)

    if setting_file_path:
        if not os.path.exists(setting_file_path):
            msg = "Setting file is not found"
            raise exceptions.ResourceNotFoundError(msg)

        with open(setting_file_path) as setting_file:
            personal_setting = json.load(setting_file)

            update_setting(personal_setting)


def get_entry_by_name(name):
    """Get a resource configuration by `name` from a robot setting dictionary.

    Args:
        name (str): A target resource name.
    Raises:
        hsrb_interface.exceptions.ResourceNotFoundError: No such resource.
    """
    for section, entries in _SETTINGS.items():
        for key, config in entries.items():
            if name == key:
                return section, config
    msg = "Item {0} is not found".format(name)
    raise exceptions.ResourceNotFoundError(msg)


def get_section(section):
    """Get a `section` from a robot setting dictionary.

    Returns:
        Dict[str, JSON Data]: A section data.
    """
    return _SETTINGS.get(section, None)


def get_entry(section, name):
    """Get an entry in robot setting dictionary.

    Args:
        section (str): A section name.
        name (str): A resource name.

    Returns:
        Dict[str, JSON Data]: A corresponding settings.

    Raises:
        hsrb_interface.exceptions.ResourceNotFoundError:
            A resource which has name `name` does not exist.
    """
    if section in _SETTINGS:
        result = _SETTINGS[section].get(name, None)
        if result is None:
            msg = "{0}({1}) is not found".format(section, name)
            raise exceptions.ResourceNotFoundError()
        else:
            return result
    else:
        msg = "{0}({1}) is not found".format(section, name)
        raise exceptions.ResourceNotFoundError(msg)


def get_frame(name):
    """Get an acutal frame id from user-friendly `name`.

    Args:
        name (str): Target frame name.

    Returns:
        str: An actual frame id.
    """
    return get_entry('frame', name)['frame_id']
