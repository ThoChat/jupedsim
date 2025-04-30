# Copyright © 2012-2024 Forschungszentrum Jülich GmbH
# SPDX-License-Identifier: LGPL-3.0-or-later
from dataclasses import dataclass

import jupedsim.native as py_jps


@dataclass(kw_only=True)
class HumanoidModelV0:
    """Parameters for Social Force Model

    All attributes are initialized with reasonably good defaults.

    See the scientific publication for more details about this model
    https://doi.org/10.1038/35035023

    Attributes:
        bodyForce: describes the strength with which an agent is influenced by pushing forces from obstacles and neighbors in its direct proximity. [in kg s^-2] (is called k)
        friction: describes the strength with which an agent is influenced by frictional forces from obstacles and neighbors in its direct proximity. [in kg m^-1 s^-1] (is called :math:`\kappa`)
    """

    bodyForce: float = 120000  # [kg s^-2] is called k
    friction: float = 240000  # [kg m^-1 s^-1] is called kappa


@dataclass(kw_only=True)
class HumanoidModelV0AgentParameters:
    """
    Parameters required to create an Agent in the Social Force Model.

    See the scientific publication for more details about this model
    https://doi.org/10.1038/35035023

    Attributes:
        position: Position of the agent.
        orientation: Orientation of the agent.
        journey_id: Id of the journey the agent follows.
        stage_id: Id of the stage the agent targets.
        velocity: current velocity of the agent.
        mass: mass of the agent. [in kg] (is called m)
        desiredSpeed: desired Speed of the agent. [in m/s] (is called v0)
        reactionTime: reaction Time of the agent. [in s] (is called :math:`\\tau`)
        agentScale: indicates how strong an agent is influenced by pushing forces from neighbors. [in N] (is called A)
        obstacleScale: indicates how strong an agent is influenced by pushing forces from obstacles. [in N] (is called A)
        forceDistance: indicates how much the distance between an agent and obstacles or neighbors influences social forces. [in m] (is called B)
        radius: radius of the space an agent occupies. [in m] (is called r)
        height: height of the agent. [in m]
        head_position: Vector of the cartesian coordinates of the head position of this agent. [in m]
        head_velocity: Velocity vector of the head this agent. [in m/s]
        shoulder_rotation_angle_z: shoulder rotation angle along the longitudinal axis (z) of this agent. [in rad]
        shoulder_rotation_velocity_z: shoulder rotation velocity along the longitudinal axis (z) of this agent. [in rad/s]
        trunk_rotation_angle_x: trunk rotation angle along the frontal axis (x) of this agent. [in rad]
        trunk_rotation_velocity_x: trunk rotation velocity along the frontal axis (x) of this agent. [in rad/s]
        trunk_rotation_angle_y: trunk rotation angle along the frontal axis (x) of this agent. [in rad]
        trunk_rotation_velocity_y: trunk rotation velocity along the sagittal axis (y) of this agent. [in rad/s]
        heel_right_position: Vector of the cartesian coordinates of the right heel position of this agent. [in m]
        heel_right_velocity: Velocity vector of the right heel this agent. [in m/s]
        heel_left_position: Vector of the cartesian coordinates of the left heel position of this agent. [in m]
        heel_left_velocity: Velocity vector of the left heel this agent. [in m/s]
    """

    # todo write force equation from paper
    position: tuple[float, float] = (0.0, 0.0)
    orientation: tuple[float, float] = (0.0, 0.0)
    journey_id: int = -1
    stage_id: int = -1
    velocity: tuple[float, float] = (0.0, 0.0)
    # the values are from paper. doi is in class description
    mass: float = 80.0  # [kg] is called m
    desiredSpeed: float = (
        0.8  # [m / s] is called v0 can be set higher depending on situation
    )
    reactionTime: float = 0.5  # [s] is called tau
    agentScale: float = 2000  # [N] is called A
    obstacleScale: float = 2000  # [N] is called A
    forceDistance: float = 0.08  # [m] is called B
    radius: float = (
        0.3  # [m] in paper 2r is uniformy distibuted in interval [0.5 m, 0.7 m]
    )
    ## Parameters for Humanoid body
    height: float = 1.7  # [m]
    ## Variables for gait
    step_timer: int = 0  # [number of time steps]
    stepping_foot_index: int = (
        1  # -1 == right foot stepping/left foot support, 0 == double stance, 1 == left foot stepping/right foot support
    )
    step_target: tuple[float, float] = (0.0, 0.0)
    ## Variables for body parts
    head_position: tuple[float, float] = (0.0, 0.0)
    head_velocity: tuple[float, float] = (0.0, 0.0)
    shoulder_rotation_angle_z: float = 0.0
    shoulder_rotation_velocity_z: float = 0.0
    trunk_rotation_angle_x: float = 0.0
    trunk_rotation_angle_y: float = 0.0
    trunk_rotation_velocity_x: float = 0.0
    trunk_rotation_velocity_y: float = 0.0
    heel_right_position: tuple[float, float] = (0.0, 0.0)
    heel_right_velocity: tuple[float, float] = (0.0, 0.0)
    heel_left_position: tuple[float, float] = (0.0, 0.0)
    heel_left_velocity: tuple[float, float] = (0.0, 0.0)

    def as_native(
        self,
    ) -> py_jps.HumanoidModelV0AgentParameters:
        return py_jps.HumanoidModelV0AgentParameters(
            position=self.position,
            orientation=self.orientation,
            journey_id=self.journey_id,
            stage_id=self.stage_id,
            velocity=self.velocity,
            mass=self.mass,
            desiredSpeed=self.desiredSpeed,
            reactionTime=self.reactionTime,
            agentScale=self.agentScale,
            obstacleScale=self.obstacleScale,
            forceDistance=self.forceDistance,
            radius=self.radius,
            height=self.height,
            step_timer=self.step_timer,
            stepping_foot_index=self.stepping_foot_index,
            step_target=self.step_target,
            head_position=self.head_position,
            head_velocity=self.head_velocity,
            shoulder_rotation_angle_z=self.shoulder_rotation_angle_z,
            shoulder_rotation_velocity_z=self.shoulder_rotation_velocity_z,
            trunk_rotation_angle_x=self.trunk_rotation_angle_x,
            trunk_rotation_angle_y=self.trunk_rotation_angle_y,
            trunk_rotation_velocity_x=self.trunk_rotation_velocity_x,
            trunk_rotation_velocity_y=self.trunk_rotation_velocity_y,
            heel_right_position=self.heel_right_position,
            heel_right_velocity=self.heel_right_velocity,
            heel_left_position=self.heel_left_position,
            heel_left_velocity=self.heel_left_velocity,
        )


class HumanoidModelV0State:
    def __init__(self, backing) -> None:
        self._obj = backing

    @property
    def velocity(self) -> tuple[float, float]:
        """velocity of this agent."""
        return self._obj.test_value

    @velocity.setter
    def velocity(self, velocity):
        self._obj.velocity = velocity

    @property
    def mass(self) -> float:
        """mass of this agent."""
        return self._obj.mass

    @mass.setter
    def mass(self, mass):
        self._obj.mass = mass

    @property
    def desiredSpeed(self) -> float:
        """desired Speed of this agent."""
        return self._obj.desiredSpeed

    @desiredSpeed.setter
    def desiredSpeed(self, desiredSpeed):
        self._obj.desiredSpeed = desiredSpeed

    @property
    def reactionTime(self) -> float:
        """reaction Time of this agent."""
        return self._obj.reactionTime

    @reactionTime.setter
    def reactionTime(self, reactionTime):
        self._obj.reactionTime = reactionTime

    @property
    def agentScale(self) -> float:
        """agent Scale of this agent."""
        return self._obj.agentScale

    @agentScale.setter
    def agentScale(self, agentScale):
        self._obj.agentScale = agentScale

    @property
    def obstacleScale(self) -> float:
        """obstacle Scale of this agent."""
        return self._obj.obstacleScale

    @obstacleScale.setter
    def obstacleScale(self, obstacleScale):
        self._obj.obstacleScale = obstacleScale

    @property
    def forceDistance(self) -> float:
        """force Distance of this agent."""
        return self._obj.forceDistance

    @forceDistance.setter
    def forceDistance(self, forceDistance):
        self._obj.forceDistance = forceDistance

    @property
    def radius(self) -> float:
        """radius of this agent."""
        return self._obj.radius

    @radius.setter
    def radius(self, radius):
        self._obj.radius = radius

    @property
    def height(self) -> float:
        """height of this agent."""
        return self._obj.height

    @height.setter
    def height(self, height):
        self._obj.height = height

    @property
    def step_timer(self) -> int:
        """number of time step before completing the current step."""
        return self._obj.step_timer

    @step_timer.setter
    def step_timer(self, step_timer):
        self._obj.step_timer = step_timer

    @property
    def stepping_foot_index(self) -> int:
        """-1 == right foot stepping/left foot support,
        0 == double stance,
        1 == left foot stepping/right foot support."""
        return self._obj.stepping_foot_index

    @stepping_foot_index.setter
    def stepping_foot_index(self, stepping_foot_index):
        self._obj.stepping_foot_index = stepping_foot_index

    @property
    def step_target(self) -> tuple[float, float]:
        """target position of the current stepping foot."""
        return self._obj.step_target

    @step_target.setter
    def step_target(self, step_target):
        self._obj.step_target = step_target

    @property
    def head_position(self) -> tuple[float, float]:
        """head position of this agent."""
        return self._obj.head_position

    @head_position.setter
    def head_position(self, head_position):
        self._obj.head_position = head_position

    @property
    def head_velocity(self) -> tuple[float, float]:
        """head velocity of this agent."""
        return self._obj.head_velocity

    @head_velocity.setter
    def head_velocity(self, head_velocity):
        self._obj.head_velocity = head_velocity

    @property
    def shoulder_rotation_angle_z(self) -> float:
        """right shoulder rotation angle z of this agent."""
        return self._obj.shoulder_rotation_angle_z

    @shoulder_rotation_angle_z.setter
    def shoulder_rotation_angle_z(self, shoulder_rotation_angle_z):
        self._obj.shoulder_rotation_angle_z = shoulder_rotation_angle_z

    @property
    def shoulder_rotation_velocity_z(self) -> float:
        """right shoulder rotation velocity z of this agent."""
        return self._obj.shoulder_rotation_velocity_z

    @shoulder_rotation_velocity_z.setter
    def shoulder_rotation_velocity_z(self, shoulder_rotation_velocity_z):
        self._obj.shoulder_rotation_velocity_z = shoulder_rotation_velocity_z

    @property
    def trunk_rotation_angle_x(self) -> float:
        """trunk rotation angle x of this agent."""
        return self._obj.trunk_rotation_angle_x

    @trunk_rotation_angle_x.setter
    def trunk_rotation_angle_x(self, trunk_rotation_angle_x):
        self._obj.trunk_rotation_angle_x = trunk_rotation_angle_x

    @property
    def trunk_rotation_angle_y(self) -> float:
        """trunk rotation angle y of this agent."""
        return self._obj.trunk_rotation_angle_y

    @trunk_rotation_angle_y.setter
    def trunk_rotation_angle_y(self, trunk_rotation_angle_y):
        self._obj.trunk_rotation_angle_y = trunk_rotation_angle_y

    @property
    def trunk_rotation_velocity_x(self) -> float:
        """trunk rotation velocity x of this agent."""
        return self._obj.trunk_rotation_velocity_x

    @trunk_rotation_velocity_x.setter
    def trunk_rotation_velocity_x(self, trunk_rotation_velocity_x):
        self._obj.trunk_rotation_velocity_x = trunk_rotation_velocity_x

    @property
    def trunk_rotation_velocity_y(self) -> float:
        """trunk rotation velocity y of this agent."""
        return self._obj.trunk_rotation_velocity_y

    @trunk_rotation_velocity_y.setter
    def trunk_rotation_velocity_y(self, trunk_rotation_velocity_y):
        self._obj.trunk_rotation_velocity_y = trunk_rotation_velocity_y

    @property
    def heel_right_position(self) -> tuple[float, float]:
        """right heel position of this agent."""
        return self._obj.heel_right_position

    @heel_right_position.setter
    def heel_right_position(self, heel_right_position):
        self._obj.heel_right_position = heel_right_position

    @property
    def heel_right_velocity(self) -> tuple[float, float]:
        """right heel velocity of this agent."""
        return self._obj.heel_right_velocity

    @heel_right_velocity.setter
    def heel_right_velocity(self, heel_right_velocity):
        self._obj.heel_right_velocity = heel_right_velocity

    @property
    def heel_left_position(self) -> tuple[float, float]:
        """left heel position of this agent."""
        return self._obj.heel_left_position

    @heel_left_position.setter
    def heel_left_position(self, heel_left_position):
        self._obj.heel_left_position = heel_left_position

    @property
    def heel_left_velocity(self) -> tuple[float, float]:
        """left heel velocity of this agent."""
        return self._obj.heel_left_velocity

    @heel_left_velocity.setter
    def heel_left_velocity(self, heel_left_velocity):
        self._obj.heel_left_velocity = heel_left_velocity
