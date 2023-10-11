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

"""Unittest hsrb_interface_py.robot module."""
from unittest.mock import patch

import hsrb_interface_py
import hsrb_interface_py.robot

from nose.tools import eq_
from nose.tools import ok_
from nose.tools import raises


@raises(hsrb_interface_py.exceptions.RobotConnectionError)
def test_resource():
    """Test resource acquisition."""
    robot = hsrb_interface_py.robot.Item()
    ok_(robot)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rclpy.node.Node.__init__')
@patch('rclpy.node.Node.destroy_node')
def test_robot_lifecycle_close(mock_destroy, mock_init,
                               mock_buffer, mock_listener):
    """Test basic lifecycle"""
    ok_(mock_buffer)
    ok_(mock_listener)
    robot = hsrb_interface_py.Robot()
    mock_init.assert_called_with('hsrb_interface_py')
    eq_(robot.ok(), True)
    robot.close()
    mock_destroy.assert_called()
    eq_(robot.ok(), False)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rclpy.node.Node.__init__')
@patch('rclpy.node.Node.destroy_node')
def test_robot_lifecycle(mock_destroy, mock_init,
                         mock_buffer, mock_listener):
    """Test use in with statement"""
    ok_(mock_buffer)
    ok_(mock_listener)
    with hsrb_interface_py.Robot() as robot:
        eq_(robot.ok(), True)
        mock_init.assert_called_with('hsrb_interface_py')
    mock_destroy.assert_called()


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('tf2_ros.BufferClient')
@patch('rclpy.node.Node.__init__')
@patch('rclpy.node.Node.destroy_node')
def test_robot_with_tf_client(mock_destroy, mock_init,
                              mock_buffer_client, mock_buffer, mock_listener):
    """Test use in tf client"""
    with hsrb_interface_py.Robot(use_tf_client=True) as robot:
        eq_(robot.ok(), True)
        mock_buffer_client.assert_called_with('/tf2_buffer_server')
        mock_buffer.assert_not_called()
        mock_listener.assert_not_called()

    mock_destroy.assert_called()
