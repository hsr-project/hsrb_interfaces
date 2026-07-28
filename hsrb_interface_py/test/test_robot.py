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
"""Unittest hsrb_interface.robot module."""
import unittest
from unittest.mock import patch

import hsrb_interface
import hsrb_interface.robot


class RobotTest(unittest.TestCase):

    def setUp(self):
        patcher = patch('tf2_ros.TransformListener')
        self.listener_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('tf2_ros.Buffer')
        self.buffer_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('tf2_ros.BufferClient')
        self.buffer_client_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('rclpy.node.Node.__init__')
        self.init_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch('rclpy.node.Node.destroy_node')
        self.destroy_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_resource(self):
        """Test resource acquisition."""
        with self.assertRaises(hsrb_interface.exceptions.RobotConnectionError):
            robot = hsrb_interface.robot.Item()
            assert robot

    def test_robot_lifecycle_close(self):
        """Test basic lifecycle"""
        assert self.buffer_mock
        assert self.listener_mock
        robot = hsrb_interface.Robot()
        self.init_mock.assert_called_with('hsrb_interface_py')
        self.assertTrue(robot.ok())
        robot.close()
        self.destroy_mock.assert_called()
        self.assertFalse(robot.ok())

    def test_robot_lifecycle(self):
        """Test use in with statement"""
        assert self.buffer_mock
        assert self.listener_mock
        with hsrb_interface.Robot() as robot:
            self.assertTrue(robot.ok())
            self.init_mock.assert_called_with('hsrb_interface_py')
        self.destroy_mock.assert_called()

    def test_robot_with_tf_client(self):
        """Test use in tf client"""
        with hsrb_interface.Robot(use_tf_client=True) as robot:
            self.assertTrue(robot.ok())
            self.buffer_mock.assert_not_called()
            self.listener_mock.assert_not_called()

        self.destroy_mock.assert_called()
