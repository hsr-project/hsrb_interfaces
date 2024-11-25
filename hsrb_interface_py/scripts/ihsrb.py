#!/usr/bin/env python3
'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
import math  # noqa : F401

from hsrb_interface import geometry  # noqa : F401
from hsrb_interface import Robot
from hsrb_interface import robot as _robot
from IPython.terminal.embed import InteractiveShellEmbed

import rclpy

from rclpy.signals import SignalHandlerOptions

try:
    from traitlets.config.loader import Config
except ImportError:
    from IPython.config.loader import Config
try:
    get_ipython
except NameError:
    nested = 0
    cfg = Config()
else:
    cfg = Config()
    nested = 1


shell = InteractiveShellEmbed(config=cfg,
                              banner1="HSR-B Interactive Shell 0.2.0",
                              exit_msg="Leaving HSR-B Interactive Shell")

LOGO = r"""
      ____________  ______  _________       __  _______ ____
     /_  __/ __ \ \/ / __ \/_  __/   |     / / / / ___// __ \
      / / / / / /\  / / / / / / / /| |    / /_/ /\__ \/ /_/ /
     / / / /_/ / / / /_/ / / / / ___ |   / __  /___/ / _, _/
    /_/  \____/ /_/\____/ /_/ /_/  |_|  /_/ /_//____/_/ |_|
"""


_robot.enable_interactive()

# main


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    with Robot() as robot:
        whole_body = robot.try_get('whole_body')  # noqa : F841
        omni_base = robot.try_get('omni_base')  # noqa : F841
        # collision_world = robot.try_get('global_collision_world')  # noqa : F841
        gripper = robot.try_get('gripper')  # noqa : F841
        tts = robot.try_get('default', robot.Items.TEXT_TO_SPEECH)  # noqa : F841
        shell(LOGO)


if __name__ == '__main__':
    main()
