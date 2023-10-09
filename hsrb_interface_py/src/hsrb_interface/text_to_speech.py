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

# vim: fileencoding=utf-8
"""Text-to-speech interface"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import rospy
from tmc_msgs.msg import Voice

from . import exceptions
from . import robot
from . import settings


class TextToSpeech(robot.Item):
    """Abstract interface for text-to-speech service

    Examples:

        .. sourcecode:: python

            with Robot() as robot:
                tts = robot.get("default", Items.TEXT_TO_SPEECH)
                tts.language = tts.JAPANESE
                tts.say(u"Hello, World!")
    """

    JAPANESE = Voice.kJapanese
    ENGLISH = Voice.kEnglish

    def __init__(self, name):
        """Initialize an instance

        Args:
            name (str): A resource name
        """
        super(TextToSpeech, self).__init__()
        self._setting = settings.get_entry('text_to_speech', name)
        topic = self._setting['topic']
        self._pub = rospy.Publisher(topic, Voice, queue_size=0)
        self._language = TextToSpeech.JAPANESE

    @property
    def language(self):
        """(int): Language of speech"""
        return self._language

    @language.setter
    def language(self, value):
        if value not in (Voice.kJapanese, Voice.kEnglish):
            msg = "Language code {0} is not supported".format(value)
            raise exceptions.InvalidLanguageError(msg)
        self._language = value

    def say(self, text):
        """Speak a given text

        Args:
            text (str): A text to be converted to voice sound (UTF-8)
        Returns:
            None
        """
        msg = Voice()
        msg.interrupting = False
        msg.queueing = False
        msg.language = self._language
        msg.sentence = text
        self._pub.publish(msg)
