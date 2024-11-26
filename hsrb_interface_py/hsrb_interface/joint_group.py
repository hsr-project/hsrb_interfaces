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
# vim: fileencoding=utf-8
"""This module contains classes and functions to move joints."""

import asyncio
import math
import sys
import warnings

from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import TransformStamped
from hsrb_interface_py._extension import KinematicsInterface
from moveit_msgs.msg import MoveItErrorCodes

import numpy as np

import rclpy

from sensor_msgs.msg import JointState

import tf2_ros
import tf_transformations as T

from tmc_manipulation_msgs.msg import BaseMovementType

from tmc_planning_msgs.msg import JointPosition
from tmc_planning_msgs.msg import TaskSpaceRegion
from tmc_planning_msgs.srv import PlanWithHandGoals
from tmc_planning_msgs.srv import PlanWithHandLine
from tmc_planning_msgs.srv import PlanWithJointGoals
from tmc_planning_msgs.srv import PlanWithTsrConstraints

import urdf_parser_py.urdf as urdf


from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from . import collision_world
from . import exceptions
from . import geometry
from . import robot
from . import settings
from . import trajectory
from . import utils


_DEBUG = False

# Timeout for motion planning [sec]
_PLANNING_ARM_TIMEOUT = 30.0

# Max number of iteration of moition planning
_PLANNING_MAX_ITERATION = 100000

# Goal generation probability in moition planning
_PLANNING_GOAL_GENERATION = 0.3

# Goal deviation in motion planning
_PLANNING_GOAL_DEVIATION = 0.3

# Timeout to receive a tf message [sec]
_TF_TIMEOUT = 5.0


def _normalize_np(vec):
    """Normalize 1D numpy.ndarray

    Args:
        vec (numpy.ndarray): A vector to be normalized
    Returns:
        numpy.ndarray: The reuslt of computation
    """
    length = np.linalg.norm(vec)
    if length < sys.float_info.epsilon:
        return vec
    else:
        vec = vec / length
        return vec


def _pose_from_x_axis(axis):
    """Compute a transformation that fits X-axis of its frame to given vector.

    Args:
        axis (geometry.Vector3): A target vector

    Returns:
        geometry.Pose: The result transformation that stored in Pose type.
    """
    axis = np.array(axis, dtype='float64', copy=True)
    axis = _normalize_np(axis)
    unit_x = np.array([1, 0, 0])
    outerp = np.cross(unit_x, axis)
    theta = math.acos(np.dot(unit_x, axis))
    if np.linalg.norm(outerp) < sys.float_info.epsilon:
        outerp = np.array([0, 1, 0])
    outerp = _normalize_np(outerp)
    q = T.quaternion_about_axis(theta, outerp)
    return geometry.Pose(geometry.Vector3(0, 0, 0), geometry.Quaternion(*q))


def _movement_axis_and_distance(pose1, pose2):
    """Compute a vector from the origin of pose1 to pose2 and distance.

    Args:
        pose1 (geometry.Pose): A pose that its origin is used as start.
        pose2 (geometry.Pose): A pose that its origin is used as goal.
    Returns:
        Tuple[geometry.Vector3, float]: The result
    """
    p1 = pose1[0]
    p2 = pose2[0]
    """Normalize Vector3"""
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    z = p2[2] - p1[2]
    length = math.sqrt(x * x + y * y + z * z)
    if length < sys.float_info.epsilon:
        return geometry.Vector3(0.0, 0.0, 0.0), 0.0
    else:
        x /= length
        y /= length
        z /= length
        return geometry.Vector3(x, y, z), length


def _invert_pose(pose):
    """Invert a given pose as if it is a transformation.

    Args:
        pose (geometry.Pose): A pose to be inverted.q
    Returns:
        geometry.Pose: The result of computation
    """
    m = T.compose_matrix(translate=pose[0],
                         angles=T.euler_from_quaternion(pose[1]))
    (_, _, euler, trans, _) = T.decompose_matrix(T.inverse_matrix(m))
    q = T.quaternion_from_euler(euler[0], euler[1], euler[2])
    return geometry.Pose(geometry.Vector3(*trans), geometry.Quaternion(*q))


class JointGroup(robot.Item):
    """Abstract interface to control a group of joints.

    Attributes:
        joint_names (List[str]):
            A list of available joints.
        joint_positions (Dict[str, float]):
            Current joint positions.
        joint_states (sensor_msgs.msg.JointState):
            A latest joint states.
        joint_limits (Dict[str, float]):
            Joint limits of a robot.
        collision_world (hsrb_interface.collsion_world.CollisionWorld):
            A present collision world to check collision.
            If None, collision checking is disabled.
        linear_weight (float):
            How much laying weight on linear movement of a mobile base.
            This attirbute affect a output trajectory of motion planning.
        angular_weight (float):
            How much laying weight on angular movement of a mobile base.
            This attirbute affect a output trajectory of motion planning.
        joint_weights (dict):
            How much laying weight on each joint of robot.
            This attirbute affect a output trajectory of motion planning.
        planning_timeout (float):
            Timeout for motion planning [sec].
        impedance_config (str):
            A name of impedance control preset config.
            If None, impeance control is disabled.
            Default is None.
        use_base_timeopt (bool):
            If true, time-optimal filter is applied to a base trajectory.
        looking_hand_constraint (bool):
            If true, the robot hand is in the robot view after the execution
            of move_end_effector_*.bot
        constraint_tsrs (List[TaskSpaceRegion]):
            A list of constraint of task space region.
    """

    def __init__(self, name):
        """See class docstring."""
        super(JointGroup, self).__init__()
        self._setting = settings.get_entry('joint_group', name)
        self._position_control_clients = []
        arm_config = self._setting['arm_controller_prefix']
        self._position_control_clients.append(
            trajectory.TrajectoryController(arm_config))
        head_config = self._setting['head_controller_prefix']
        self._position_control_clients.append(
            trajectory.TrajectoryController(head_config))
        """
        hand_config = self._setting["hand_controller_prefix"]
        self._position_control_clients.append(
            trajectory.TrajectoryController(hand_config,self._node ))
        """
        base_config = self._setting["omni_base_controller_prefix"]
        self._base_client = trajectory.TrajectoryController(
            base_config, "base_coordinates")
        self._position_control_clients.append(self._base_client)
        """
        imp_config = settings.get_entry("trajectory", "impedance_control")
        self._impedance_client = trajectory.ImpedanceController(imp_config,self._node )
        """
        joint_state_topic = self._setting["joint_states_topic"]
        self._joint_state_sub = utils.CachingSubscriber(
            joint_state_topic,
            JointState,
            default=JointState())
        self._joint_state_sub.wait_for_message(3.0)
        self._tf2_buffer = robot._get_tf2_buffer()
        self._end_effector_frames = self._setting['end_effector_frames']
        self._end_effector_frame = self._end_effector_frames[0]
        self._passive_joints = self._setting['passive_joints']
        urdf_string = utils.get_parameters_from_another_node(
            self._node, "/robot_rrt_planner_node/get_parameters", ["robot_description"])
        self._robot_urdf = urdf.URDF.from_xml_string(urdf_string[0])
        self._kinematics_interface = KinematicsInterface(urdf_string[0])
        self._collision_world = None
        self._linear_weight = 3.0
        self._angular_weight = 1.0
        self._joint_weights = {}
        self._planning_timeout = _PLANNING_ARM_TIMEOUT
        self._use_base_timeopt = True
        self._looking_hand_constraint = False
        self._constraint_tsrs = []
        self._tf_timeout = _TF_TIMEOUT

        if _DEBUG:
            self._vis_pub = self.create_publisher(MarkerArray, "tsr_marker", 1)
            self._tf2_pub = tf2_ros.TransformBroadcaster()

    def _get_joint_state(self):
        """Get a current joint state.

        Returns:
            sensor_msgs.JointState: Current joint state
        """
        return self._joint_state_sub.data

    @property
    def joint_names(self):
        return self._get_joint_state().name

    @property
    def joint_positions(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_velocities(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.velocity))

    @property
    def joint_state(self):
        return self._get_joint_state()

    @property
    def joint_limits(self):
        joint_map = self._robot_urdf.joint_map
        return {joint_name: (joint_map[joint_name].limit.lower,
                             joint_map[joint_name].limit.upper)
                for joint_name in self.joint_names}

    @property
    def collision_world(self):
        return self._collision_world

    @collision_world.setter
    def collision_world(self, value):
        if value is None:
            self._collision_world = None
        elif isinstance(value, collision_world.CollisionWorld):
            self._collision_world = value
        else:
            raise TypeError("value should be CollisionWorld instance")

    @property
    def linear_weight(self):
        return self._linear_weight

    @linear_weight.setter
    def linear_weight(self, value):
        f_value = float(value)
        if f_value > 0.0:
            self._linear_weight = f_value
        else:
            raise ValueError("value should be positive")

    @property
    def joint_weights(self):
        return self._joint_weights

    @joint_weights.setter
    def joint_weights(self, value):
        if not isinstance(value, dict):
            raise ValueError("value should be dictionary")
        for key, weight in value.items():
            if key not in self._setting['motion_planning_joints']:
                raise ValueError(key + " is not in motion planning joints")
            if float(weight) <= 0.0:
                raise ValueError("weight should be positive")
        self._joint_weights = {key: float(weight)
                               for key, weight in value.items()}

    @property
    def angular_weight(self):
        return self._angular_weight

    @angular_weight.setter
    def angular_weight(self, value):
        f_value = float(value)
        if f_value > 0.0:
            self._angular_weight = f_value
        else:
            raise ValueError("value should be positive")

    @property
    def planning_timeout(self):
        return self._planning_timeout

    @planning_timeout.setter
    def planning_timeout(self, value):
        f_value = float(value)
        if f_value > 0.0:
            self._planning_timeout = f_value
        else:
            raise ValueError("value should be positive")

    @property
    def impedance_config(self):
        return self._impedance_client.config

    @impedance_config.setter
    def impedance_config(self, value):
        self._impedance_client.config = value

    @property
    def impedance_config_names(self):
        return self._impedance_client.config_names

    @property
    def use_base_timeopt(self):
        return self._use_base_timeopt

    @use_base_timeopt.setter
    def use_base_timeopt(self, value):
        self._use_base_timeopt = value

    @property
    def tf_timeout(self):
        return self._tf_timeout

    @tf_timeout.setter
    def tf_timeout(self, value):
        self._tf_timeout = float(value)

    @property
    def end_effector_frame(self):
        """Get or set the target end effector frame of motion planning.

        This attribute affects behaviors of following methods:
        * get_end_effector_pose
        * move_end_effector_pose
        * move_end_effector_by_line
        """
        return self._end_effector_frame

    @end_effector_frame.setter
    def end_effector_frame(self, value):
        if value in set(self._end_effector_frames):
            self._end_effector_frame = value
        else:
            msg = "`ref_frame_id` must be one of end-effector frames({0})"
            raise ValueError(msg.format(self._end_effector_frames))

    @property
    def end_effector_frames(self):
        return tuple(self._end_effector_frames)

    @property
    def looking_hand_constraint(self):
        return self._looking_hand_constraint

    @looking_hand_constraint.setter
    def looking_hand_constraint(self, value):
        self._looking_hand_constraint = value

    @property
    def constraint_tsrs(self):
        return self._constraint_tsrs

    @constraint_tsrs.setter
    def constraint_tsrs(self, value):
        if not isinstance(value, list):
            raise ValueError("value should be list")
        for constraint_tsr in value:
            if not isinstance(constraint_tsr, TaskSpaceRegion):
                raise TypeError("value should be TaskSpaceRegion instance")
        self._constraint_tsrs = value

    def _set_constraint_tsrs(self, req):
        if len(self._constraint_tsrs) > 0:
            # It is important to put TSR here
            # Since this request variable is also found in hands goals, etc., TSR constraints should be added in the same way.
            req.constraint_tsrs = self._constraint_tsrs
            # If you restrict only the posture transition, it is necessary that the bogie can move.
            # This process is not required except for the gaze transition
            #
            # The IK is solving in the middle of the process, but if it is less than 6 flexibility, IK itself becomes an error.
            # In the first place, I felt weird to solve the IK for constraints in CBirrt2, but what do you do?
            # If you want to implement without using IK, comment out +
            # In ConstraintotSr of tmc_manipulation_planner/tmc_robot_planner/src/Robot_cbirrt_planner.cpp
            # In the else part of CalcDistanceTotsR, suddenly Return False; +
            # _Planning_max_iteration increase by about a single digit
            # It is unknown which IK is used/uses, and which is better
            #
            if req.base_movement_type.val is BaseMovementType.NONE:
                # PLANWITHJOINTGOALSREQUEST only _generate_planning_request is set to None
                req.base_movement_type.val = BaseMovementType.RAIL_X
                req.weighted_joints = ['_linear_base']
                req.weight = [100.0]
        return req

    def _change_joint_state(self, joint_names, joint_positions_seq):
        """Move joints to specified joint state while checking self collision.

        Args:
            joint_names (List[str]):
                A list of joint name.
            joint_positions_seq (List[List[float]]):
                A list of target position [m or rad].
        Returns:
            None
        Raises:
            ValueError: Some specified joints are not found.
            ValueError: Target joints include some uncontrollable joints.
            ValueError: The number of joint_names and joint_positions are different.
        """
        # Validate joint names
        initial_joint_state = self._get_joint_state()
        active_joint_set = set(initial_joint_state.name)
        target_joint_set = set(joint_names)
        if not target_joint_set.issubset(active_joint_set):
            unknown_set = target_joint_set.difference(active_joint_set)
            msg = "No such joint(s): [{0}]".format(', '.join(unknown_set))
            raise ValueError(msg)
        if 'base_roll_joint' in target_joint_set:
            raise ValueError(
                "base_roll_joint is not supported in change_joint_state")
        passive_joint_set = set(self._passive_joints)
        if not target_joint_set.isdisjoint(passive_joint_set):
            intersected = target_joint_set.intersection(passive_joint_set)
            msg = "Passive joint(s): [{0}]".format(', '.join(intersected))
            raise ValueError(msg)

        req = self._generate_planning_request(PlanWithJointGoals.Request)
        req.use_joints = joint_names
        for joint_positions in joint_positions_seq:
            if len(joint_names) != len(joint_positions):
                raise ValueError("The number of joint_names and joint_positions are different")
            goal_position = JointPosition()
            goal_position.position = joint_positions
            req.goal_joint_states.append(goal_position)

        req = self._set_constraint_tsrs(req)

        service_name = self._setting['plan_with_joint_goals_service']
        plan_service = self._node.create_client(
            PlanWithJointGoals, service_name)
        future = plan_service.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            msg = "Fail to plan change_joint_state"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution)
        self._execute_trajectory(constrained_traj)

    def move_to_joint_positions_multiple_targets(self, joint_names, joint_positions_seq):
        """Move joints to goal positions with constraint.

        Args:
            joint_names (List[str]):
                A list of joint name.
            joint_positions_seq (List[List[float]]):
                A list of target position [m or rad].

        Returns:
            None

        Raises:
            ValueError: Some specified joints are not found.
            ValueError: Target joints include some uncontrollable joints.
            ValueError: The number of joint_names and joint_positions are different.

        See Also:
            :py:attr:`.joint_names`

        Examples:

            .. sourcecode:: python

                import math
                import hsrb_interface

                with hsrb_interface.Robot() as robot:
                    whole_body = robot.get('whole_body')

                    joint_names = [
                        'arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint',
                        'wrist_flex_joint', 'wrist_roll_joint'
                    ]
                    joint_positions_seq = [
                        [0.0, 0.0, -1.57, -1.57, 0.0],
                        [-0.3, 0.0, 0.0, -1.72, 1.57],
                        [-0.3, 0.0, 0.0, -1.72, -1.57]
                    ]

                    whole_body.move_to_joint_positions_multiple_targets(
                        joint_names,
                        joint_positions_seq
                    )

        """
        if not joint_names:
            return

        self._change_joint_state(joint_names, joint_positions_seq)

    def move_to_joint_positions(self, goals={}, **kwargs):
        """Move joints to a specified goal positions.

        Args:
            goals (Dict[str, float]):
                A dict of pair of joint name and target position [m or rad].
            **kwargs:
                Use keyword arguments to specify joint_name/posiion pairs.
                The keyword arguments overwrite `goals` argument.

        Returns:
            None

        See Also:
            :py:attr:`.joint_names`

        Examples:

            .. sourcecode:: python

               import math
               import hsrb_interface

               with hsrb_interface.Robot() as robot:
                   whole_body = robot.get('whole_body')
                   goals = {
                       'arm_lift_joint': 0.5,
                       'arm_flex_joint': math.radians(-90)
                   }
                   whole_body.move_to_joint_positions(goals)

                   # The method also accept keyword arguments
                   whole_body.move_to_joint_positions(
                       head_tilt_joint=math.radians(30)
                   )

        """
        if goals is None:
            goals = {}
        goals.update(kwargs)
        if not goals:
            return
        joint_names = []
        joint_positions = []
        for k, v in goals.items():
            joint_names.append(k)
            joint_positions.append(v)
        self._change_joint_state(joint_names, [joint_positions])

    def move_to_neutral(self):
        """Move joints to neutral(initial) pose of a robot."""
        goals = {
            'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0,
        }
        self.move_to_joint_positions(goals)

    def move_to_go(self):
        """Move joints to a suitable pose for moving a mobile base."""
        goals = {
            'arm_flex_joint': 0.0,
            'arm_lift_joint': 0.0,
            'arm_roll_joint': -1.57,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
        }
        self.move_to_joint_positions(goals)

    def get_end_effector_pose(self, ref_frame_id=None):
        """Get a pose of end effector based on robot frame.

        Returns:
            Tuple[Vector3, Quaternion]
        """
        # Default reference frame is a robot frame
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')

        tf_future = self._tf2_buffer.wait_for_transform_async(
            target_frame=ref_frame_id,
            source_frame=self._end_effector_frame,
            time=rclpy.time.Time()
        )
        rclpy.spin_until_future_complete(
            self._node, tf_future, timeout_sec=self._tf_timeout)

        transform = asyncio.run(self._tf2_buffer.lookup_transform_async(
            self._end_effector_frame,
            ref_frame_id,
            rclpy.time.Time()
        ))

        return geometry.transform_to_tuples(transform.transform)

    def _lookup_odom_to_ref(self, ref_frame_id):
        """Resolve current reference frame transformation from ``odom``.

        Returns:
            geometry_msgs.msg.Pose:
                A transform from robot ``odom`` to ``ref_frame_id``.
        """
        tf_future = self._tf2_buffer.wait_for_transform_async(
            target_frame=settings.get_frame('odom'),
            source_frame=ref_frame_id,
            time=rclpy.time.Time()
        )
        rclpy.spin_until_future_complete(
            self._node, tf_future, timeout_sec=self._tf_timeout)

        odom_to_ref_ros = asyncio.run(self._tf2_buffer.lookup_transform_async(
            settings.get_frame('odom'),
            ref_frame_id,
            rclpy.time.Time()
        ))
        odom_to_ref_tuples = geometry.transform_to_tuples(
            odom_to_ref_ros.transform)
        return geometry.tuples_to_pose(odom_to_ref_tuples)

    def move_end_effector_pose(self, pose, ref_frame_id=None):
        """Move an end effector to a given pose.

        Args
            pose (geometry.Pose or list(geometry.Pose)):
                The target pose(s) of the end effector frame.
            ref_frame_id (str): A base frame of an end effector.
                The default is the robot frame(```base_footprint``).
        Returns:
            None
        """
        # Default is the robot frame (the base frame)
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')

        if isinstance(pose, list):
            ref_to_hand_poses = pose
        else:
            ref_to_hand_poses = [pose]

        odom_to_ref_pose = self._lookup_odom_to_ref(ref_frame_id)
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_hand_poses = []
        for ref_to_hand in ref_to_hand_poses:
            odom_to_hand = geometry.multiply_tuples(odom_to_ref, ref_to_hand)
            odom_to_hand_poses.append(geometry.tuples_to_pose(odom_to_hand))

        req = self._generate_planning_request(PlanWithHandGoals.Request)
        req.origin_to_hand_goals = odom_to_hand_poses
        req.ref_frame_id = self._end_effector_frame

        req = self._set_constraint_tsrs(req)

        service_name = self._setting['plan_with_hand_goals_service']
        plan_service = self._node.create_client(
            PlanWithHandGoals, service_name)
        future = plan_service.call_async(req)
        res = utils.wait_until_complete(self._node, future)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            msg = "Fail to plan move_endpoint(" + str(res.error_code.val) + ")"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution)
        self._execute_trajectory(constrained_traj)

    def move_end_effector_by_line(self, axis, distance, ref_frame_id=None):
        """Move an end effector along with a line in a 3D space.

        Args:
            axis (Vector3): A axis to move along with
            distance (float): Distance to move [m]
            ref_frame_id (str):
                [DEPRECATED] The frame name of the target end effector.
                ``axis`` is defined on this frame.
        Returns:
            None
        """
        axis_length = np.linalg.norm(np.array(axis, dtype='float64'))
        if axis_length < sys.float_info.epsilon:
            raise ValueError("The axis is zero vector.")
        if ref_frame_id is None:
            end_effector_frame = self._end_effector_frame
        else:
            msg = ' '.join(["`ref_frame_id` argument is deprecated."
                            "Use `end_effector_frame` attribute instead."])
            warnings.warn(msg, exceptions.DeprecationWarning)
            if ref_frame_id not in self._end_effector_frames:
                msg = "ref_frame_id must be one of end-effector frames({0})"
                raise ValueError(msg.format(self._end_effector_frames))
            else:
                end_effector_frame = ref_frame_id

        req = self._generate_planning_request(PlanWithHandLine.Request)
        req.axis.x = float(axis[0])
        req.axis.y = float(axis[1])
        req.axis.z = float(axis[2])
        req.local_origin_of_axis = True
        req.ref_frame_id = end_effector_frame
        req.goal_value = distance

        service_name = self._setting['plan_with_hand_line_service']
        plan_service = self._node.create_client(PlanWithHandLine, service_name)
        future = plan_service.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            msg = "Fail to plan move_hand_line"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution)
        self._execute_trajectory(constrained_traj)

    def move_end_effector_by_arc(self, center, angle, ref_frame_id=None):
        """Move an end effector along with an arc in a 3D space.

        Args:
            center (Tuple[Vector3, Quaternion]):
                A center pose of rotation. The z axis is used as rotation axis.
            angle (float): Angle to move [rad]. The range is (-PI, PI)
            ref_frame_id (str): A base frame of an end effector.
                The default is the robot frame(```base_footprint``).
        Returns:
            None
        """
        # Check angle value
        if not -math.pi < angle < math.pi:
            raise ValueError("The range of the angle is (-PI, PI).")

        # Default is the robot frame (the base frame)
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')

        odom_to_ref_pose = self._lookup_odom_to_ref(ref_frame_id)
        odom_to_ref = geometry.pose_to_tuples(odom_to_ref_pose)
        odom_to_center = geometry.multiply_tuples(odom_to_ref, center)

        center_to_odom = _invert_pose(odom_to_center)
        odom_to_hand_pose = self._lookup_odom_to_ref(self._end_effector_frame)
        odom_to_hand = geometry.pose_to_tuples(odom_to_hand_pose)
        center_to_hand = geometry.multiply_tuples(center_to_odom, odom_to_hand)

        rotation_tsr = TaskSpaceRegion()
        rotation_tsr.end_frame_id = self.end_effector_frame
        rotation_tsr.origin_to_tsr = geometry.tuples_to_pose(odom_to_center)
        rotation_tsr.tsr_to_end = geometry.tuples_to_pose(center_to_hand)
        if angle < 0:
            rotation_tsr.min_bounds = [0.0, 0.0, 0.0, 0.0, 0.0, angle]
            rotation_tsr.max_bounds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            rotation_tsr.min_bounds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            rotation_tsr.max_bounds = [0.0, 0.0, 0.0, 0.0, 0.0, angle]

        goal_tsr = TaskSpaceRegion()
        goal_tsr.end_frame_id = self.end_effector_frame
        goal_tsr.origin_to_tsr = geometry.tuples_to_pose(odom_to_center)
        goal_tsr.tsr_to_end = geometry.tuples_to_pose(center_to_hand)
        goal_tsr.min_bounds = [0.0, 0.0, 0.0, 0.0, 0.0, angle]
        goal_tsr.max_bounds = [0.0, 0.0, 0.0, 0.0, 0.0, angle]

        req = self._generate_planning_request(PlanWithTsrConstraints.Request)
        req.constraint_tsrs = [rotation_tsr]
        req.goal_tsrs = [goal_tsr]

        service_name = self._setting['plan_with_constraints_service']
        plan_service = self._node.create_client(
            PlanWithTsrConstraints, service_name)
        future = plan_service.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            msg = "Fail to plan"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        res.base_solution.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(res.solution,
                                                        res.base_solution)
        self._execute_trajectory(constrained_traj)

    def _plan_cartesian_path(self, origin_to_pose1, origin_to_pose2,
                             odom_to_robot_pose,
                             initial_joint_state,
                             collision_env):
        req = self._generate_planning_request(PlanWithTsrConstraints.Request)
        req.origin_to_basejoint = odom_to_robot_pose
        req.initial_joint_state = initial_joint_state
        if collision_env is not None:
            req.environment_before_planning = collision_env
        req.extra_constraints = []
        req.extra_goal_constraints = []

        move_axis, distance = _movement_axis_and_distance(origin_to_pose1,
                                                          origin_to_pose2)
        origin_to_axis = _pose_from_x_axis(move_axis)
        pose1_to_axis = geometry.multiply_tuples(_invert_pose(origin_to_pose1),
                                                 origin_to_axis)
        pose1_to_axis = geometry.Pose((0, 0, 0), pose1_to_axis[1])
        origin_to_tsr = geometry.multiply_tuples(
            origin_to_pose1, pose1_to_axis)
        tsr_to_pose1 = _invert_pose(pose1_to_axis)

        if _DEBUG:
            print("AXIS     ", move_axis)
            print("DISTANCE ", distance)
            print("ORIGIN   ", origin_to_pose1)
            print("CALCUD   ", geometry.multiply_tuples(
                origin_to_tsr, tsr_to_pose1))

            margin = 0.1
            marker_pose = geometry.multiply_tuples(origin_to_tsr,
                                                   geometry.pose(
                                                       x=distance / 2.0))
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self._node.get_clock().now().to_msg()
            marker.ns = 'tsr'
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = marker_pose[0][0]
            marker.pose.position.y = marker_pose[0][1]
            marker.pose.position.z = marker_pose[0][2]
            marker.pose.orientation.x = marker_pose[1][0]
            marker.pose.orientation.y = marker_pose[1][1]
            marker.pose.orientation.z = marker_pose[1][2]
            marker.pose.orientation.w = marker_pose[1][3]
            marker.scale.x = distance
            marker.scale.y = margin
            marker.scale.z = margin
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            marker.lifetime = rclpy.duration.Duration(seconds=60.0).to_msg()
            msg = MarkerArray(markers=[marker])
            self._vis_pub.publish(msg)

            t = TransformStamped()
            t.header.stamp = self._node.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "pose1"
            t.transform.translation.x = origin_to_pose1[0][0]
            t.transform.translation.y = origin_to_pose1[0][1]
            t.transform.translation.z = origin_to_pose1[0][2]
            t.transform.rotation.x = origin_to_pose1[1][0]
            t.transform.rotation.y = origin_to_pose1[1][1]
            t.transform.rotation.z = origin_to_pose1[1][2]
            t.transform.rotation.w = origin_to_pose1[1][3]
            self._tf2_pub.sendTransform(t)

            t = TransformStamped()
            t.header.stamp = self._node.get_clock().now().to_msg()
            t.header.frame_id = "pose1"
            t.child_frame_id = "direction"
            t.transform.translation.x = pose1_to_axis[0][0]
            t.transform.translation.y = pose1_to_axis[0][1]
            t.transform.translation.z = pose1_to_axis[0][2]
            t.transform.rotation.x = pose1_to_axis[1][0]
            t.transform.rotation.y = pose1_to_axis[1][1]
            t.transform.rotation.z = pose1_to_axis[1][2]
            t.transform.rotation.w = pose1_to_axis[1][3]
            self._tf2_pub.sendTransform(t)

            t = TransformStamped()
            t.header.stamp = self._node.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "pose2"
            t.transform.translation.x = origin_to_pose2[0][0]
            t.transform.translation.y = origin_to_pose2[0][1]
            t.transform.translation.z = origin_to_pose2[0][2]
            t.transform.rotation.x = origin_to_pose2[1][0]
            t.transform.rotation.y = origin_to_pose2[1][1]
            t.transform.rotation.z = origin_to_pose2[1][2]
            t.transform.rotation.w = origin_to_pose2[1][3]
            self._tf2_pub.sendTransform(t)

            t = TransformStamped()
            t.header.stamp = self._node.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "tsr"
            t.transform.translation.x = origin_to_tsr[0][0]
            t.transform.translation.y = origin_to_tsr[0][1]
            t.transform.translation.z = origin_to_tsr[0][2]
            t.transform.rotation.x = origin_to_tsr[1][0]
            t.transform.rotation.y = origin_to_tsr[1][1]
            t.transform.rotation.z = origin_to_tsr[1][2]
            t.transform.rotation.w = origin_to_tsr[1][3]
            self._tf2_pub.sendTransform(t)

        # Goal constraint
        tsr_g = TaskSpaceRegion()
        tsr_g.end_frame_id = bytes(self.end_effector_frame)
        tsr_g.origin_to_tsr = geometry.tuples_to_pose(origin_to_pose2)
        tsr_g.tsr_to_end = geometry.tuples_to_pose(geometry.pose())
        tsr_g.min_bounds = [0, 0, 0, 0, 0, 0]
        tsr_g.max_bounds = [0, 0, 0, 0, 0, 0]

        # Line constraint
        tsr_c = TaskSpaceRegion()
        tsr_c.end_frame_id = bytes(self.end_effector_frame)
        tsr_c.origin_to_tsr = geometry.tuples_to_pose(origin_to_tsr)
        tsr_c.tsr_to_end = geometry.tuples_to_pose(tsr_to_pose1)
        tsr_c.min_bounds = [0, 0, 0, -math.pi, -math.pi, -math.pi]
        tsr_c.max_bounds = [distance, 0, 0, math.pi, math.pi, math.pi]

        req.goal_tsrs = [tsr_g]
        req.constraint_tsrs = [tsr_c]

        service_name = self._setting['plan_with_constraints_service']

        plan_service = self._node.create_client(
            PlanWithTsrConstraints, service_name)
        future = plan_service.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            msg = "Fail to plan"
            raise exceptions.MotionPlanningError(msg, res.error_code)
        return res

    def move_cartesian_path(self, waypoints, ref_frame_id=None):
        """Move the end-effector along a path that follows specified waypoints.

        Args:
            waypoints (List[Pose]): End effector poses
            ref_frame_id (str):
                The base frame of waypoints (default is the robot frame)
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        base_frame = settings.get_frame('base')
        origin_to_ref_ros_pose = self._lookup_odom_to_ref(ref_frame_id)
        origin_to_ref = geometry.pose_to_tuples(origin_to_ref_ros_pose)
        origin_to_pose1 = self.get_end_effector_pose('odom')
        odom_to_robot_pose = self._lookup_odom_to_ref(base_frame)
        initial_joint_state = self._get_joint_state()
        if self._collision_world is not None:
            collision_env = self._collision_world.snapshot('odom')
        else:
            collision_env = None

        arm_traj = None
        base_traj = None

        for i in range(len(waypoints)):
            origin_to_pose2 = geometry.multiply_tuples(origin_to_ref,
                                                       waypoints[i])
            plan = self._plan_cartesian_path(origin_to_pose1,
                                             origin_to_pose2,
                                             odom_to_robot_pose,
                                             initial_joint_state,
                                             collision_env)
            if arm_traj is None:
                arm_traj = plan.solution
            elif len(plan.solution.points) > 0:
                arm_traj.points.extend(plan.solution.points[1:])
            if base_traj is None:
                base_traj = plan.base_solution
            elif len(plan.base_solution.points) > 0:
                base_traj.points.extend(plan.base_solution.points[1:])

            origin_to_pose1 = origin_to_pose2
            odom_to_robot_pose = RosPose()
            final_transform = plan.base_solution.points[-1].transforms[0]
            odom_to_robot_pose.position.x = final_transform.translation.x
            odom_to_robot_pose.position.y = final_transform.translation.y
            odom_to_robot_pose.position.z = final_transform.translation.z
            odom_to_robot_pose.orientation.x = final_transform.rotation.x
            odom_to_robot_pose.orientation.y = final_transform.rotation.y
            odom_to_robot_pose.orientation.z = final_transform.rotation.z
            odom_to_robot_pose.orientation.w = final_transform.rotation.w
            initial_joint_state = plan.joint_state_after_planning

        base_traj.header.frame_id = settings.get_frame('odom')
        constrained_traj = self._constrain_trajectories(arm_traj, base_traj)
        self._execute_trajectory(constrained_traj)

    def gaze_point(self, point=geometry.vector3(), ref_frame_id=None):
        """Point the rgbd sensor at given place.

        Args:
            point (Vector3): A position to point.
            ref_frame_id (str): A base frame of the point.
                The default is the robot frame(```base_footprint``).
        Returns:
            None
        Notes:
            If the calculated angle is over the limit, the angle is rounded.
        """
        if np.isinf(point).any() or np.isnan(point).any():
            raise ValueError("The point includes inf or nan.")
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        origin_to_ref_ros_pose = self._lookup_odom_to_ref(ref_frame_id)
        origin_to_ref = geometry.pose_to_tuples(origin_to_ref_ros_pose)

        origin_to_base_ros_pose = self._lookup_odom_to_ref(
            settings.get_frame('base'))
        origin_to_base = geometry.pose_to_tuples(origin_to_base_ros_pose)
        base_to_origin = _invert_pose(origin_to_base)

        ref_to_point = geometry.Pose(point, geometry.quaternion())

        base_to_ref = geometry.multiply_tuples(base_to_origin, origin_to_ref)
        base_to_point = geometry.multiply_tuples(base_to_ref, ref_to_point)

        result = self._kinematics_interface.calculate_gazing_angles(
            list(base_to_point.pos), str(self._setting['rgbd_sensor_frame']))
        if len(result) == 0:
            raise RuntimeError("Cannot gaze the given point.")
        self.move_to_joint_positions(result)

    def _generate_planning_request(self, request_type):
        """Generate a planning request and assign common parameters to it.

        Args:
            request_type (Types):
                A type of "planning service request".
                The following are available types.
                    - tmc_planning_msgs.srv.PlanWithHandGoalsRequest
                    - tmc_planning_msgs.srv.PlanWithHandLineRequest
                    - tmc_planning_msgs.srv.PlanWithJointGoalsRequest
                    - tmc_planning_msgs.srv.PlanWithTsrConstraintsRequest

        Retruns:
            tmc_planning_msgs.srv.PlanWithXXX:
                An instance with common parameters.
        """
        request = request_type()
        request.origin_to_basejoint = self._lookup_odom_to_ref(
            settings.get_frame('base'))
        request.initial_joint_state = self._get_joint_state()
        request.timeout = rclpy.duration.Duration(
            seconds=self._planning_timeout).to_msg()
        request.max_iteration = _PLANNING_MAX_ITERATION
        if self._collision_world is not None:
            snapshot = self._collision_world.snapshot(
                settings.get_frame('odom'))
            request.environment_before_planning = snapshot

            request.attached_objects = self._collision_world.attached_objects

            # Request.environment_before_planning is not included in Attached_objects
            # Since the operation plan fails, add it here
            for attached_object in self._collision_world.attached_objects:
                already_known_object_flag = False
                for known_object in request.environment_before_planning.collision_objects:
                    if known_object.id == attached_object.object.id:
                        already_known_object_flag = True
                        break

                if not already_known_object_flag:
                    request.environment_before_planning.collision_objects.append(attached_object.object)

        if request_type is PlanWithJointGoals.Request:
            request.base_movement_type.val = BaseMovementType.NONE
            return request
        else:
            use_joints = set(['wrist_flex_joint',
                              'wrist_roll_joint',
                              'arm_roll_joint',
                              'arm_flex_joint',
                              'arm_lift_joint'])
            if self._looking_hand_constraint:
                use_joints.update(
                    self._setting['looking_hand_constraint']['use_joints'])
                request.extra_goal_constraints.append(
                    self._setting['looking_hand_constraint']['plugin_name'])
            request.use_joints = use_joints
            request.base_movement_type.val = BaseMovementType.PLANAR
            request.uniform_bound_sampling = False
            request.deviation_for_bound_sampling = _PLANNING_GOAL_DEVIATION
            request.probability_goal_generate = _PLANNING_GOAL_GENERATION
            request.weighted_joints = ['_linear_base', '_rotational_base']
            request.weighted_joints.extend(self._joint_weights.keys())
            request.weight = [self._linear_weight, self._angular_weight]
            request.weight.extend(self._joint_weights.values())
            return request

    def _constrain_trajectories(self, joint_trajectory, base_trajectory=None):
        """Apply constraints to given trajectories.

        Parameters:
            joint_trajectory (trajectory_msgs.msg.JointTrajectory):
                A upper body trajectory
            base_trajectory (trajectory_msgs.msg.JointTrajectory):
                A base trajectory
        Returns:
            trajectory_msgs.msg.JointTrajectory:
                A constrained trajectory
        Raises:
            TrajectoryFilterError:
                Failed to execute trajectory-filtering
        """
        if base_trajectory:
            odom_base_trajectory = trajectory.transform_base_trajectory(
                base_trajectory, self._tf2_buffer, self._tf_timeout,
                self._base_client.joint_names, self._node)
            merged_traj = trajectory.merge(joint_trajectory,
                                           odom_base_trajectory)
        else:
            merged_traj = joint_trajectory

        filtered_merged_traj = None
        if self._use_base_timeopt:
            start_state = self.joint_state
            # use traj first point for odom
            if base_trajectory:
                start_state.name += self._base_client.joint_names
                start_state.position += \
                    odom_base_trajectory.points[0].positions
            filtered_merged_traj = trajectory.hsr_timeopt_filter(
                merged_traj, start_state, self._node)
        return filtered_merged_traj

    def _execute_trajectory(self, joint_traj):
        """Execute a trajectory with given action clients.

        Action clients that actually execute trajectories are selected
        automatically.

        Parameters:
            joint_traj (trajectory_msgs.msg.JointTrajectory):
                A trajectory to be executed
        Returns:
            None
        """
        clients = []
        if False:  # self._impedance_client.config is not None:
            clients.append(self._impedance_client)
        else:
            for client in self._position_control_clients:
                for joint in joint_traj.joint_names:
                    if joint in client.joint_names:
                        clients.append(client)
                        break
        joint_states = self._get_joint_state()

        for client in clients:
            traj = trajectory.extract(joint_traj, client.joint_names,
                                      joint_states)
            client.submit(traj)

        trajectory.wait_controllers(self._node, clients)
