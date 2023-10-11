#!/usr/bin/env python3
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


import math  # noqa

from hsrb_interface_py import geometry  # noqa
from hsrb_interface_py import Robot
from hsrb_interface_py import robot as _robot

from IPython.terminal.embed import InteractiveShellEmbed

import rclpy

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


def main():
    _robot.enable_interactive()

    rclpy.init()
    with Robot() as robot:
        whole_body = robot.try_get('whole_body')  # noqa
        # omni_base = robot.try_get('omni_base')
        # collision_world = robot.try_get('global_collision_world')
        # suction = robot.try_get('suction')
        gripper = robot.try_get('gripper')  # noqa
        # wrist_wrench = robot.try_get('wrist_wrench')
        # marker = robot.try_get('marker')
        # battery = robot.try_get('battery')
        # tts = robot.try_get('default_tts')
        shell(LOGO)
