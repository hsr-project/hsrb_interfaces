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

"""Unittest for sensor objects."""
from geometry_msgs.msg import WrenchStamped
import hsrb_interface.sensors
from mock import patch
from nose.tools import eq_
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_camera(mock_get_entry, mock_connecting, mock_sub_class):
    """Test Camera class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': 'head_l_stereo_camera',
        'prefix': "/stereo_camera/left",
    }

    camera = hsrb_interface.sensors.Camera('example')
    mock_get_entry.assert_called_with('camera', 'example')

    mock_sub_class.assert_called_with("/stereo_camera/left/image_raw", Image)
    mock_sub_instance = mock_sub_class.return_value

    msg = Image()
    msg.header.stamp.secs = 1
    msg.header.stamp.nsecs = 2
    msg.header.seq = 3
    msg.header.frame_id = 'map'
    msg.height = 16
    msg.width = 16
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = range(256)

    mock_sub_instance.data = msg
    image = camera.image
    eq_(image.to_ros(), msg)


@patch('rospy.ServiceProxy')
@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_force_torque(mock_get_entry, mock_connecting,
                      mock_sub_class, mock_service):
    """Test ForceTorque class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': "example",
        'raw_topic': "raw_wrench",
        'compensated_topic': "compensated_wrench",
        'reset_service': "reset_wrench",
    }

    force_torque = hsrb_interface.sensors.ForceTorque('example')
    mock_get_entry.assert_called_with('force_torque', 'example')

    mock_sub_class.assert_any_call("raw_wrench", WrenchStamped)
    mock_sub_class.assert_any_call("compensated_wrench", WrenchStamped)

    mock_sub_instance = mock_sub_class.return_value

    msg = WrenchStamped()
    msg.wrench.force.x = 0
    msg.wrench.force.y = 1
    msg.wrench.force.z = 2
    msg.wrench.torque.x = 3
    msg.wrench.torque.y = 4
    msg.wrench.torque.z = 5
    mock_sub_instance.data = msg

    wrench = force_torque.raw
    eq_(wrench, ((0, 1, 2), (3, 4, 5)))
    wrench = force_torque.wrench
    eq_(wrench, ((0, 1, 2), (3, 4, 5)))

    eq_(None, force_torque.reset())
    mock_service.assert_called_with("reset_wrench", Empty)


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_imu(mock_get_entry, mock_connecting, mock_sub_class):
    """Test Imu class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': "example",
        'topic': "foo",
    }

    imu = hsrb_interface.sensors.IMU('example')
    mock_get_entry.assert_called_with('imu', 'example')

    mock_sub_class.assert_called_with("foo", Imu)
    mock_sub_instance = mock_sub_class.return_value

    msg = Imu()
    msg.orientation.x = 0
    msg.orientation.y = 1
    msg.orientation.z = 2
    msg.orientation.w = 3
    msg.angular_velocity.x = 4
    msg.angular_velocity.y = 5
    msg.angular_velocity.z = 6
    msg.linear_acceleration.x = 7
    msg.linear_acceleration.y = 8
    msg.linear_acceleration.z = 9

    mock_sub_instance.data = msg

    ori, angular_vel, linear_acc = imu.data
    eq_(ori, (0, 1, 2, 3))
    eq_(angular_vel, (4, 5, 6))
    eq_(linear_acc, (7, 8, 9))


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.robot.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_lidar(mock_get_entry, mock_connecting, mock_sub_class):
    """Test Lidar class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': "example",
        'topic': "foo",
    }

    lidar = hsrb_interface.sensors.Lidar('example')
    mock_get_entry.assert_called_with('lidar', 'example')

    mock_sub_class.assert_called_with("foo", LaserScan)
    mock_sub_instance = mock_sub_class.return_value

    msg = LaserScan()
    mock_sub_instance.data = msg

    scan = lidar.scan

    eq_(scan.to_ros(), msg)
