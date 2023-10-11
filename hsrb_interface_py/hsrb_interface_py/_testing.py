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

"""Testing Utilities"""

import math
import unittest
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped
import hsrb_interface_py
from hsrb_interface_py import geometry

import hsrb_interface_py.robot

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.duration import Duration
import tf2_ros


def dict_almost_equal(a, b, delta):
    """Compare 2 Dict[str, float] instances."""
    keys = a.keys()
    if sorted(keys) == sorted(b.keys()):
        return all(abs(a[key] - b[key]) < delta for key in keys)
    else:
        raise ValueError("a and b don't share keys")


def vector3_distance(v1, v2):
    """Compute distance of 2 vectors."""
    return math.sqrt(sum((a - b) ** 2.0 for a, b in zip(v1, v2)))


def quaternion_distance(q1, q2):
    """Compute shortest angle distance in 2 quaternions."""
    product = sum(a * b for a, b in zip(q1, q2))
    return abs(math.acos(2.0 * product ** 2.0 - 1.0))


class RosMockTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize class variables"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Disconnect from a robot."""
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("hsrb_interface_test")
        patcher = patch("hsrb_interface_py.Robot._connection")
        self.connection_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.connection_mock.return_value = self.node

        patcher = patch("hsrb_interface_py.Robot._connecting")
        self.robot_connecting_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.robot_connecting_mock = True

        patcher = patch("hsrb_interface_py.settings.get_entry")
        self.get_entry_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface_py.settings.get_frame")
        self.get_frame_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("rclpy.node.Node.get_clock")
        self.time_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # patcher = patch("self.node.create_publisher")
        # self.publisher_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        patcher = patch("rclpy.node.Node.create_subscription")
        self.subscriber_mock = patcher.start()
        self.addCleanup(patcher.stop)

        # patcher = patch("rospy.ServiceProxy")
        # self.service_proxy_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        # patcher = patch("self.node.create_client")
        # self.client_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        # patcher = patch("self.node.get_parameter")
        # self.get_param_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        patcher = patch("rclpy.action.ActionClient")
        self.action_client_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def tearDown(self):
        self.node.destroy_node()


class HsrbInterfaceTest(unittest.TestCase):
    """Base class for simulation tests."""

    EXPECTED_NEUTRAL = {
        "arm_lift_joint": 0,
        "arm_flex_joint": 0,
        "arm_roll_joint": 0,
        "wrist_flex_joint": math.radians(-90),
        "wrist_roll_joint": 0,
        "head_pan_joint": 0,
        "head_tilt_joint": 0,
    }

    EXPECTED_TO_GO = {
        "arm_lift_joint": 0,
        "arm_flex_joint": 0,
        "arm_roll_joint": math.radians(-90),
        "wrist_flex_joint": math.radians(-90),
        "wrist_roll_joint": 0,
        "head_pan_joint": 0,
        "head_tilt_joint": 0,
    }

    JOINT_NAMES = [
        'arm_flex_joint',
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
        'wrist_roll_joint'
    ]

    BASE_MOVE_TIME_TOLERANCE = 60
    BASE_MOVE_GOAL_TOLERANCE = 0.05

    @classmethod
    def setUpClass(cls):
        """Initialize class variables"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Disconnect from a robot."""
        rclpy.shutdown()

    def setUp(self):
        # Wait until simluation clock starts
        clock = Clock(clock_type=ClockType.ROS_TIME)
        now = clock.now()
        while now == rclpy.time.Time().to_msg():
            now = clock.now()
        self.robot = hsrb_interface_py.Robot()
        now = self.robot.node.get_clock().now()
        self.whole_body = self.robot.get('whole_body')
        # self.omni_base = self.robot.get('omni_base')
        self.gripper = self.robot.get('gripper')
        # self.collision_world = self.robot.get('global_collision_world')
        # self.marker = self.robot.get('marker')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(
            self.tf_buffer, self.robot.node)

    def tearDown(self):
        self.robot.close()

    def expect_pose_equal(self, pose1, pose2, delta=None):
        """Check elements of pose1 and pose2 are almost equal.

        Args:
            pose1 (Tuple[Vector3, Quaternion]): A pose to be compared.
            pose2 (Tuple[Vector3, Quaternion]): The other pose.
            delta (float): If a distance of 2 pose elements is within `delta`,
                that elemment is thought as equal.
        """
        pos1, rot1 = pose1
        pos2, rot2 = pose2
        self.assertAlmostEqual(pos1[0], pos2[0], delta=delta)
        self.assertAlmostEqual(pos1[1], pos2[1], delta=delta)
        self.assertAlmostEqual(pos1[2], pos2[2], delta=delta)
        self.assertAlmostEqual(rot1[0], rot2[0], delta=delta)
        self.assertAlmostEqual(rot1[1], rot2[1], delta=delta)
        self.assertAlmostEqual(rot1[2], rot2[2], delta=delta)
        self.assertAlmostEqual(rot1[3], rot2[3], delta=delta)

    def expect_dict_contains_subset(self, a, b, delta=None):
        """For common keys in `a` and `b`, check ||a[key] - b[key]|| < delta.

        Args:
            a (Dict[Any, float]): A dict tobe compared.
            b (Dict[Any, float]): The other dict.
            delta (float): Equality threshold.
        """
        for key in a:
            if key not in b:
                self.fail()
            else:
                self.assertAlmostEqual(a[key], b[key], delta=delta)

    def expect_joints_reach_goals(self, expected, delta=None,
                                  timeout=30.0, tick=0.5):
        """Success if all `expected` joints reach their goals within `timeout`.

        Args:
            expected (Dict[str, float]): Target joints and their goals.
            delta (float): Threshold to a joint reach its goal.
            timeout (float): Timeout duration in seconds.
            tick (float): Sleep duration between each check.
        """
        start = self.robot.node.get_clock().now()
        timeout = Duration(seconds=timeout)
        rate = self.robot.node.create_rate(tick)
        if delta is None:
            delta = 0
        while True:
            num_reached = 0
            for joint, goal in expected.items():
                position = self.whole_body.joint_positions[joint]
                if abs(position - goal) <= delta:
                    num_reached += 1
            now = self.robot.node.get_clock().now()
            if (now - start) > timeout:
                diffs = []
                positions = self.whole_body.joint_positions
                for key in expected:
                    template = '{0}: expected={1} actual={2}'
                    diff = template.format(key, expected[key], positions[key])
                    diffs.append(diff)
                self.fail('Timed out: {0}'.format(', '.join(diffs)))
            if num_reached == len(expected):
                break
            else:
                rate.sleep()

    def expect_gripper_to_grasp(self, delta=None):
        """Check a robot suceeded to grasp within `timeout`.

        Args:
            delta (Optional[float]): Error tolerance(0.0 if If None)
        """
        motor = self.whole_body.joint_positions['hand_motor_joint']
        right = self.whole_body.joint_positions['hand_r_spring_proximal_joint']
        left = self.whole_body.joint_positions['hand_l_spring_proximal_joint']

        if delta is None:
            delta = 0

        self.assertNotAlmostEqual(motor, right, delta=delta)
        self.assertNotAlmostEqual(motor, left, delta=delta)

    def expect_hand_reach_goal(self, goal, frame='map',
                               pos_delta=None, ori_delta=None,
                               timeout=30.0, tick=0.5):
        """Check a robot move its hand to a given goal withing `timeout`

        Args:
            goal (): Expected goal pose
            frame (str): base frame of `goal`
            pos_delta (float): Position tolerance [m]
            ori_delta (float): Orientation tolerance
                (Measured in closest angle difference) [rad]
            timeout (float): Timeout in seconds
            tick (float): Sleep time between goal check in seconds
        """
        start = self.robot.node.get_clock().now()
        timeout = Duration(seconds=timeout)
        rate = self.robot.node.create_rate(tick)
        pos_delta = 0.0 if pos_delta is None else pos_delta
        ori_delta = 0.0 if ori_delta is None else ori_delta
        while True:
            pose = self.whole_body.get_end_effector_pose(frame)
            pos_error = vector3_distance(pose[0], goal[0])
            ori_error = quaternion_distance(pose[1], goal[1])

            if pos_error < pos_delta and ori_error < ori_delta:
                break
            else:
                rate.sleep()
            now = self.robot.node.get_clock().now()
            if (now - start) > timeout:
                msg = '\n'.join([
                    'Timed out:',
                    '\nexpected  = {0}'.format(goal),
                    '\nacutal    = {0}'.format(pose),
                    '\npos_error = {0} < {1}'.format(pos_error, pos_delta),
                    '\nori_error = {0} < {1}'.format(ori_error, ori_delta),
                    '\nframe     = {0}'.format(frame),
                ])
                self.fail(msg)

    def expect_base_reach_goal(self, goal, frame='map',
                               pos_delta=None, ori_delta=None,
                               timeout=30.0, tick=0.5):
        """Check a robot moves its base to a given goal withing `timeout`.

        Args:
            goal (): Expected goal pose
            frame (str): base frame of `goal`
            pos_delta (Optional[float]): Position tolerance [m]
            ori_delta (Optional[float]): Orientation tolerance
                (Measured in closest angle difference) [rad]
            timeout (float): Timeout in seconds
            tick (float): Sleep time between goal check in seconds
        """
        start = self.robot.node.get_clock().now()
        timeout = Duration(seconds=timeout)
        rate = self.robot.node.create_rate(tick)
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rclpy.time.Time().to_msg()
        goal_pose.header.frame_id = frame
        goal_pose.pose = geometry.tuples_to_pose(goal)
        map_to_goal_pose = self.tf_buffer.transform(goal_pose, 'map')
        map_to_goal_pose.pose.position.z = 0.0
        goal = geometry.pose_to_tuples(map_to_goal_pose.pose)

        pos_delta = 0.0 if pos_delta is None else pos_delta
        ori_delta = 0.0 if ori_delta is None else ori_delta
        while True:
            pose = self.omni_base.get_pose('map')
            pos_error = vector3_distance(pose[0], goal[0])
            ori_error = quaternion_distance(pose[1], goal[1])

            if pos_error < pos_delta and ori_error < ori_delta:
                break
            else:
                rate.sleep()
            now = self.robot.node.get_clock().now()
            if (now - start) > timeout:
                msg = '\n'.join([
                    'Timed out:',
                    '\texpected  = {0}'.format(goal),
                    '\tacutal    = {0}'.format(pose),
                    '\tpos_error = {0} < {1}'.format(pos_error, pos_delta),
                    '\tori_error = {0} < {1}'.format(ori_error, ori_delta),
                    '\tframe     = {0}'.format(frame),
                ])
                self.fail(msg)

    def expect_detected_object(self, object_id, expected_pose,
                               frame='map', pos_delta=None, ori_delta=None,
                               timeout=30.0, tick=0.5):
        """Check a robot is detecting an object with given ID.

        Args:
            object_id (int): Expected known object ID
            expected_pose (Tuple[Vector3, Quaternion]): Expected goal pose
            frame (str): base frame of `goal`
            pos_delta (float): Position tolerance [m]
            ori_delta (float): Orientation tolerance
                (Measured in closest angle difference) [rad]
            timeout (float): Timeout in seconds
            tick (float): Sleep time between goal check in seconds
        """
        start = self.robot.node.get_clock().now()
        timeout = Duration(seconds=timeout)
        rate = self.robot.node.create_rate(tick)

        pos_delta = 0.0 if pos_delta is None else pos_delta
        ori_delta = 0.0 if ori_delta is None else ori_delta
        while True:
            now = self.robot.node.get_clock().now()
            obj = self.marker.get_object_by_id(object_id)
            if obj is not None:
                pose = obj.get_pose(frame)
                pos_error = vector3_distance(pose[0], expected_pose[0])
                ori_error = quaternion_distance(pose[1], expected_pose[1])

                if pos_error < pos_delta and ori_error < ori_delta:
                    break
            if (now - start) > timeout:
                buf = ['Timed out:']
                if obj is not None:
                    buf.extend([
                        '\texpected  = {0}'.format(expected_pose),
                        '\tactual    = {0}'.format(pose),
                        '\tpos_error = {0} < {1}'.format(pos_error, pos_delta),
                        '\tori_error = {0} < {1}'.format(ori_error, ori_delta),
                    ])
                buf.append('\tframe     = {0}'.format(frame))
                msg = '\n'.join(buf)
                self.fail(msg)
            rate.sleep()
