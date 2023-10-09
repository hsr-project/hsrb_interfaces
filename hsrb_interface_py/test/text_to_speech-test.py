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

"""Unittest for hsrb_interface.text_to_speech module"""
import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.text_to_speech
from mock import patch
from nose.tools import eq_
from nose.tools import raises
from tmc_msgs.msg import Voice


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rospy.Publisher')
def test_text_to_speech(mock_pub_class, mock_get_entry, mock_connecting):
    """Test simple usage of TTS object."""
    mock_connecting.return_value = True

    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    mock_get_entry.return_value = {"topic": "foo"}
    mock_get_entry.called_with_args("text_to_speech", "default_tts")
    mock_pub_class.called_with_args("foo", Voice, queue_size=0)
    mock_pub_instance = mock_pub_class.return_value

    eq_(tts.language, tts.JAPANESE)
    tts.language = tts.ENGLISH
    eq_(tts.language, tts.ENGLISH)

    expected_msg = Voice()
    expected_msg.interrupting = False
    expected_msg.queueing = False
    expected_msg.language = False
    expected_msg.sentence = "Hello, World!"
    tts.say(u"Hello, World!")
    mock_pub_instance.called_with_args(expected_msg)


@raises(hsrb_interface.exceptions.InvalidLanguageError)
@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rospy.Publisher')
def test_invalid_language_error(mock_pub_class, mock_get_entry,
                                mock_connecting):
    """TTS object should refuse invalid language."""
    mock_connecting.return_value = True

    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    mock_get_entry.return_value = {"topic": "foo"}

    tts.language = -1
