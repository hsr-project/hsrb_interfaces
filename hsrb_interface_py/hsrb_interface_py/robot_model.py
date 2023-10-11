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

from urdf_parser_py import urdf
from urdf_parser_py.urdf import Robot as RobotModel
from urdf_parser_py.urdf import xmlr

# Monkeypatching urdf_parser_py (1)
# https://github.com/k-okada/urdfdom/commit/9c9e28e9f1de29228def48a4d49315b2f4fbf2d2
# If urdf_parser_py is released with merging the commit above,
# we can safely remove following code.
xmlr.reflect(urdf.JointLimit, params=[
    xmlr.Attribute('effort', float),
    xmlr.Attribute('lower', float, False, 0),
    xmlr.Attribute('upper', float, False, 0),
    xmlr.Attribute('velocity', float)
])


# Monkeypatching urdf_parser_py (2)
# URDF specification allow multiple <visual> and <collision> elements.
# http://wiki.ros.org/urdf/XML/link
xmlr.reflect(urdf.Link, params=[
    xmlr.Attribute('name', str),
    xmlr.Element('origin', urdf.Pose, False),
    xmlr.Element('inertial', urdf.Inertial, False),
    xmlr.AggregateElement('visual', urdf.Visual, 'visual'),
    xmlr.AggregateElement('collision', urdf.Collision, 'collision')
])

# Monkeypatching urdf_parser_py (3)
# <actuator> may have <hardwareInterface>.
# http://wiki.ros.org/urdf/XML/Transmission
xmlr.reflect(urdf.Actuator, tag='actuator', params=[
    xmlr.Attribute('name', str),
    xmlr.Element('mechanicalReduction', float, required=False),
    xmlr.AggregateElement('hardwareInterface', str)
])


def _get_aggregate_list(self, xml_var):
    var = self.XML_REFL.paramMap[xml_var].var
    if not getattr(self, var):
        self.aggregate_init()
        setattr(self, var, [])
    return getattr(self, var)


urdf.Link.get_aggregate_list = _get_aggregate_list
urdf.Collision.get_aggregate_list = _get_aggregate_list
urdf.Material.check_valid = lambda self: None
urdf.Actuator.hardwareInterfaces = []
urdf.Actuator.get_aggregate_list = _get_aggregate_list

__all__ = ('RobotModel',)
