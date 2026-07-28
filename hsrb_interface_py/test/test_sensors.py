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
"""Unittest for sensor objects."""
from unittest.mock import patch

import _testing as testing
from geometry_msgs.msg import WrenchStamped
from hsrb_interface import Robot
import hsrb_interface.sensors
import rclpy

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan


class SensorTest(testing.RosMockTestCase):

    def setUp(self):
        super().setUp()

        patcher = patch("hsrb_interface.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_camera(self):
        """Test Camera class"""
        rclpy.init()
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'name': 'head_l_stereo_camera',
            'prefix': "/stereo_camera/left",
        }

        camera = hsrb_interface.sensors.Camera('example')
        self.get_entry_mock.assert_called_with('camera', 'example')

        self.caching_sub_mock.assert_called_with("/stereo_camera/left/image_raw", Image)
        mock_sub_instance = self.caching_sub_mock.return_value

        msg = Image()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 2
        msg.header.frame_id = 'map'
        msg.height = 16
        msg.width = 16
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = range(256)

        mock_sub_instance.data = msg
        image = camera.image
        self.assertEqual(image.to_ros(), msg)

    def test_force_torque(self):
        """Test ForceTorque class"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'name': "example",
            'raw_topic': "raw_wrench",
            'compensated_topic': "compensated_wrench",
            'reset_service': "reset_wrench",
        }

        force_torque = hsrb_interface.sensors.ForceTorque('example')
        self.get_entry_mock.assert_called_with('force_torque', 'example')

        self.caching_sub_mock.assert_any_call("raw_wrench", WrenchStamped)
        self.caching_sub_mock.assert_any_call("compensated_wrench", WrenchStamped)

        mock_sub_instance = self.caching_sub_mock.return_value

        msg = WrenchStamped()
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 1.0
        msg.wrench.force.z = 2.0
        msg.wrench.torque.x = 3.0
        msg.wrench.torque.y = 4.0
        msg.wrench.torque.z = 5.0
        mock_sub_instance.data = msg

        wrench = force_torque.raw
        self.assertEqual(wrench, ((0.0, 1.0, 2.0), (3.0, 4.0, 5.0)))
        wrench = force_torque.wrench
        self.assertEqual(wrench, ((0.0, 1.0, 2.0), (3.0, 4.0, 5.0)))

        self.assertEqual(None, force_torque.reset())

    def test_imu(self):
        """Test Imu class"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'name': "example",
            'topic': "foo",
        }

        imu = hsrb_interface.sensors.IMU('example')
        self.get_entry_mock.assert_called_with('imu', 'example')

        self.caching_sub_mock.assert_called_with("foo", Imu)
        mock_sub_instance = self.caching_sub_mock.return_value

        msg = Imu()
        msg.orientation.x = 0.0
        msg.orientation.y = 1.0
        msg.orientation.z = 2.0
        msg.orientation.w = 3.0
        msg.angular_velocity.x = 4.0
        msg.angular_velocity.y = 5.0
        msg.angular_velocity.z = 6.0
        msg.linear_acceleration.x = 7.0
        msg.linear_acceleration.y = 8.0
        msg.linear_acceleration.z = 9.0

        mock_sub_instance.data = msg

        ori, angular_vel, linear_acc = imu.data
        self.assertEqual(ori, (0.0, 1.0, 2.0, 3.0))
        self.assertEqual(angular_vel, (4.0, 5.0, 6.0))
        self.assertEqual(linear_acc, (7.0, 8.0, 9.0))

    def test_lidar(self):
        """Test Lidar class"""
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {
            'name': "example",
            'topic': "foo",
        }

        lidar = hsrb_interface.sensors.Lidar('example')
        self.get_entry_mock.assert_called_with('lidar', 'example')

        self.caching_sub_mock.assert_called_with("foo", LaserScan)
        mock_sub_instance = self.caching_sub_mock.return_value

        msg = LaserScan()
        mock_sub_instance.data = msg

        scan = lidar.scan

        self.assertEqual(scan.to_ros(), msg)
