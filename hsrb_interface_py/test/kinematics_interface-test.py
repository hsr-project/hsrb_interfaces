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

"""Unittest for hsrb_interface._extension.KinematicsInterface module"""
import os
import unittest

from hsrb_interface._extension import KinematicsInterface
from nose.tools import assert_almost_equals
from nose.tools import assert_raises
from nose.tools import assert_true
from nose.tools import eq_
import rospkg


class KinematicsInterfaceTest(unittest.TestCase):

    def setUp(self):
        rp = rospkg.RosPack()
        urdf_xml = os.path.join(rp.get_path('hsrb_description'),
                                'robots', 'hsrb.urdf')
        f = open(urdf_xml)
        self._kinematics = KinematicsInterface(f.read())

    def test_normal(self):
        """Test normal case"""
        result = self._kinematics.calculate_gazing_angles(
            [1.0, 0.0, 0.0], 'head_rgbd_sensor_link')
        eq_(len(result), 2)
        assert_true('head_pan_joint' in result.keys())
        assert_true('head_tilt_joint' in result.keys())
        assert_almost_equals(result['head_pan_joint'], -0.06196844)
        assert_almost_equals(result['head_tilt_joint'], -0.82959372)

        result = self._kinematics.calculate_gazing_angles(
            [1.0, 0.0, 0.0], 'head_l_stereo_camera_link')
        eq_(len(result), 2)
        assert_true('head_pan_joint' in result.keys())
        assert_true('head_tilt_joint' in result.keys())
        assert_almost_equals(result['head_pan_joint'], -0.07021305)
        assert_almost_equals(result['head_tilt_joint'], -0.70391284)

        result = self._kinematics.calculate_gazing_angles(
            [1.0, 0.0, 0.0], 'head_r_stereo_camera_link')
        eq_(len(result), 2)
        assert_true('head_pan_joint' in result.keys())
        assert_true('head_tilt_joint' in result.keys())
        assert_almost_equals(result['head_pan_joint'], 0.07021305)
        assert_almost_equals(result['head_tilt_joint'], -0.70391284)

    def test_invalid_point_length(self):
        """The dimension of point must be three."""
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [], 'head_rgbd_sensor_link')
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [1.0], 'head_rgbd_sensor_link')
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [1.0, 0.0], 'head_rgbd_sensor_link')
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [1.0, 0.0, 0.0, 0.0], 'head_rgbd_sensor_link')

    def test_uncomputable_point(self):
        """If the solution is nothing, return empty dictionary."""
        result = self._kinematics.calculate_gazing_angles(
            [0.0, 0.0, 0.757], 'head_rgbd_sensor_link')
        eq_(len(result), 0)

    def test_invalid_frame(self):
        """A frame name which is not in robot model is not accepted."""
        assert_raises(RuntimeError, self._kinematics.calculate_gazing_angles,
                      [1.0, 0.0, 0.0], 'head_rgbd_sensor_frame')
