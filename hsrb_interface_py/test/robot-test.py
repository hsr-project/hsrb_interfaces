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

"""Unittest hsrb_interface.robot module."""
import hsrb_interface
import hsrb_interface.robot
from mock import patch
from nose.tools import eq_
from nose.tools import ok_
from nose.tools import raises


@raises(hsrb_interface.exceptions.RobotConnectionError)
def test_resource():
    """Test resource acquisition."""
    robot = hsrb_interface.robot.Item()
    ok_(robot)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rospy.get_master')
@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_lifecycle(init_mock, shutdown_mock, mock_get_master,
                         mock_buffer, mock_listener):
    """Test basic lifecycle"""
    ok_(mock_get_master)
    ok_(mock_buffer)
    ok_(mock_listener)
    robot = hsrb_interface.Robot()
    init_mock.assert_called_with('hsrb_interface_py', disable_signals=False,
                                 anonymous=True)
    eq_(robot.ok(), True)
    robot.close()
    shutdown_mock.assert_called_with('shutdown')
    eq_(robot.ok(), False)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rospy.get_master')
@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_with_statement(init_mock, shutdown_mock, mock_get_master,
                              mock_buffer, mock_listener):
    """Test use in with statement"""
    ok_(mock_get_master)
    ok_(mock_buffer)
    ok_(mock_listener)
    with hsrb_interface.Robot() as robot:
        eq_(robot.ok(), True)
        init_mock.assert_called_with('hsrb_interface_py',
                                     disable_signals=False, anonymous=True)
    shutdown_mock.assert_called_with('shutdown')


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('tf2_ros.BufferClient')
@patch('rospy.get_master')
@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_with_tf_client(init_mock, shutdown_mock, mock_get_master,
                              mock_buffer_client, mock_buffer, mock_listener):
    """Test use in tf client"""
    with hsrb_interface.Robot(use_tf_client=True) as robot:
        eq_(robot.ok(), True)
        mock_buffer_client.assert_called_with('/tf2_buffer_server')
        mock_buffer.assert_not_called()
        mock_listener.assert_not_called()

    shutdown_mock.assert_called_with('shutdown')
