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

"""Mapping table of ROS Graph names.

This module is intended to internal use only.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import json

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
            "constraint_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "whole_timeopt_filter_service": "/filter_hsrb_trajectory",
            "caster_joint": "base_roll_joint",
            "filter_timeout": 30.0,
            "action_timeout": 30.0,
            "watch_rate": 30.0
    },
    "joint_group": {
        "whole_body": {
            "class":                        ["joint_group", "JointGroup"],
            "joint_states_topic":           "/hsrb/joint_states",
            "arm_controller_prefix":        "/hsrb/arm_trajectory_controller",
            "head_controller_prefix":       "/hsrb/head_trajectory_controller",
            "hand_controller_prefix":       "/hsrb/gripper_controller",
            "omni_base_controller_prefix":  "/hsrb/omni_base_controller",
            "plan_with_constraints_service":"/plan_with_constraints",
            "plan_with_hand_goals_service": "/plan_with_hand_goals",
            "plan_with_hand_line_service":  "/plan_with_hand_line",
            "plan_with_joint_goals_service":"/plan_with_joint_goals",
            "timeout":                       1.0,
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
    },
    "end_effector": {
        "gripper": {
            "class":        ["end_effector", "Gripper"],
            "joint_names":  ["hand_motor_joint"],
            "prefix":       "/hsrb/gripper_controller",
            "left_finger_joint_name":  "hand_l_spring_proximal_joint",
            "right_finger_joint_name": "hand_r_spring_proximal_joint"
        },
        "suction": {
            "class": ["end_effector", "Suction"],
            "action":                         "/hsrb/suction_control",
            "suction_topic":                  "/hsrb/command_suction",
            "pressure_sensor_topic":          "/hsrb/pressure_sensor",
            "timeout":                        1.0
        }
    },
    "mobile_base": {
        "omni_base": {
            "class": ["mobile_base", "MobileBase"],
            "move_base_action":          "/move_base/move",
            "follow_trajectory_action":  "/hsrb/omni_base_controller",
            "pose_topic":                "/global_pose",
            "goal_topic":                "/base_goal",
            "timeout":                   1.0
        }
    },
    "camera": {
        "head_l_stereo_camera": {
            "class":   ["sensors", "Camera"],
            "prefix":  "/hsrb/head_l_stereo_camera",
            "timeout": 3.0
        },
        "head_r_stereo_camera": {
            "class": ["sensors", "Camera"],
            "prefix":  "/hsrb/head_l_stereo_camera",
            "timeout": 3.0
        },
        "head_rgbd_sensor_rgb": {
            "class": ["sensors", "Camera"],
            "prefix": "/hsrb/head_rgbd_sensor/rgb",
            "timeout": 3.0
        },
        "head_rgbd_sensor_depth": {
            "class": ["sensors", "Camera"],
            "prefix": "/hsrb/head_rgbd_sensor/depth_registered",
            "timeout": 3.0
        }
    },
    "imu": {
        "base_imu": {
            "class": ["sensors", "IMU"],
            "topic": "/hsrb/base_imu/data",
            "timeout": 1.0
        }
    },
    "force_torque": {
        "wrist_wrench": {
            "class": ["sensors", "ForceTorque"],
            "raw_topic": "/hsrb/wrist_wrench/raw",
            "compensated_topic": "/hsrb/wrist_wrench/compensated",
            "reset_service": "/hsrb/wrist_wrench/readjust_offset",
            "timeout": 1.0
        }
    },
    "lidar": {
        "base_scan": {
            "class": ["sensors", "Lidar"],
            "topic": "/hsrb/base_scan",
            "timeout": 1.0
        }
    },
    "object_detection": {
        "marker": {
            "class": ["object_detection", "ObjectDetector"],
            "topic": "/recognized_object"
        }
    },
    "power_supply": {
        "battery": {
            "class": ["battery", "Battery"],
            "topic": "/hsrb/battery_state",
            "timeout": 2.0
        }
    },
    "text_to_speech": {
        "default_tts": {
            "class": ["text_to_speech", "TextToSpeech"],
            "topic": "/talk_request"
        }
    },
    "collision_world": {
        "global_collision_world": {
            "class": ["collision_world", "CollisionWorld"],
            "service": "/get_collision_environment",
            "control_topic": "/known_object",
            "listing_topic": "/known_object_ids"
        }
    }
}
"""

_SETTINGS = json.loads(_HSRB_SETTINGS)


def get_entry_by_name(name):
    """Get a resource configuration by `name` from a robot setting dictionary.

    Args:
        name (str): A target resource name.

    Returns:


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
