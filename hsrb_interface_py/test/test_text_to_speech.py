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
"""Unittest for hsrb_interface.text_to_speech module"""
from unittest.mock import patch

import _testing as testing
import hsrb_interface
import hsrb_interface.exceptions
from hsrb_interface.robot import Robot
import hsrb_interface.text_to_speech
import rclpy
from tmc_voice_msgs.msg import Voice


class TextToSpeechTest(testing.RosMockTestCase):

    def setUp(self):
        super().setUp()

        patcher = patch("rclpy.node.Node.create_publisher")
        self.publisher_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_text_to_speech(self):
        """Test simple usage of TTS object."""
        rclpy.init()
        robot = Robot()  # noqa: F841

        self.get_entry_mock.return_value = {"topic": "foo"}

        tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

        self.get_entry_mock.assert_called_with("text_to_speech", "default_tts")
        self.publisher_mock.assert_called_with(Voice, "foo", 0)
        mock_pub_instance = self.publisher_mock.return_value

        self.assertEqual(tts.language, tts.JAPANESE)
        tts.language = tts.ENGLISH
        self.assertEqual(tts.language, tts.ENGLISH)

        expected_msg = Voice()
        expected_msg.interrupting = False
        expected_msg.queueing = False
        expected_msg.language = Voice.ENGLISH
        expected_msg.sentence = "Hello, World!"
        tts.say(u"Hello, World!")
        mock_pub_instance.publish.assert_called_with(expected_msg)

    def __test_invalid_language_error(self):
        """TTS object should refuse invalid language."""
        with self.assertRaises(hsrb_interface.exceptions.InvalidLanguageError):
            self.get_entry_mock.return_value = {"topic": "foo"}

            tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

            tts.language = -1
