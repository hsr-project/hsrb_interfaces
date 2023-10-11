# vim: fileencoding=utf-8
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

"""Defitions of common exceptions."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals


class HsrbInterfaceError(Exception):
    """Base class for all other exceptions"""


class RobotConnectionError(HsrbInterfaceError):
    """An error that express a failure in a connection to a robot."""


class ResourceNotFoundError(HsrbInterfaceError):
    """An error that express a specified resource does not exist."""


class TrajectoryLengthError(HsrbInterfaceError):
    """A logical error in handling trajectories."""


class FollowTrajectoryError(HsrbInterfaceError):
    """Playing back a trajectory went wrong."""


class PlannerError(HsrbInterfaceError):
    """A request to motion planner failed.

    This exception is deprecated. Use MotionPlanningError instread.
    """


class TrajectoryFilterError(HsrbInterfaceError):
    """A request to a trajectory filter service failed.

    Args:
        message (str): Error message
        error_code (ArmManipulationErrorCodes): An error code
    """

    def __init__(self, message, error_code):
        super(TrajectoryFilterError, self).__init__(message)
        self._error_code = error_code


class MotionPlanningError(PlannerError):
    """Translate a motion planning error code to a human readable text.

    Args:
        message (str): Error message
        error_code (ArmManipulationErrorCodes): An error code
    """

    def __init__(self, message, error_code):
        super(MotionPlanningError, self).__init__(message)
        self._error_code = error_code


class GripperError(HsrbInterfaceError):
    """A command to a gripper failed."""


class InvalidLanguageError(HsrbInterfaceError):
    """A TTS service does not accept a given language."""


class MobileBaseError(HsrbInterfaceError):
    """Something wrong in a mobile base."""


class DeprecationWarning(Warning):
    """Indicate a feature is deprecated."""
