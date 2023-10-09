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

"""Unittest for hsrb_interface.battery module"""

import hsrb_interface
import hsrb_interface.battery
from mock import patch
from nose.tools import eq_
from tmc_msgs.msg import BatteryState


@patch.object(hsrb_interface.Robot, '_connection')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.utils.CachingSubscriber')
def test_battery(mock_sub_class, mock_get_entry, mock_connection):
    """Test Battery class"""
    mock_connection.return_value = True
    mock_sub_instance = mock_sub_class.return_value
    mock_get_entry.return_value = {
        'topic': 'hoge'
    }
    battery = hsrb_interface.battery.Battery('foo')
    mock_get_entry.assert_called_with('power_supply', 'foo')
    mock_sub_class.assert_called_with('hoge', BatteryState)

    mock_sub_instance.data.power = 100.0
    mock_sub_instance.data.temperature = 25.0

    eq_(battery.charge, 100.0)
    eq_(battery.temperature, 25.0)
