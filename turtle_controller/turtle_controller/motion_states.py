"""State pattern for the goToPose controller.

The motion to a target pose is split into phases. Each phase is a State object
that knows how to compute a velocity command for the current pose and which
state comes next. The action server (the Context) only calls ``step()`` on the
active state, so adding or changing a phase never touches the server code.

    RotateToTarget -> DriveToTarget -> RotateToYaw -> Reached
          ^                 |
          +-----------------+  (heading drifted too far while driving)
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from turtle_controller.geometry import clamp, distance, normalize_angle


@dataclass
class ControlContext:
    """Everything a state needs to compute one command."""

    x: float
    y: float
    theta: float
    goal_x: float
    goal_y: float
    goal_yaw: float
    max_linear: float
    max_angular: float
    k_linear: float
    k_angular: float
    position_tolerance: float
    yaw_tolerance: float
    heading_tolerance: float

    @property
    def distance_error(self) -> float:
        return distance(self.x, self.y, self.goal_x, self.goal_y)

    @property
    def heading_error(self) -> float:
        """Angle between the turtle heading and the direction to the target."""
        bearing = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        return normalize_angle(bearing - self.theta)

    @property
    def yaw_error(self) -> float:
        return normalize_angle(self.goal_yaw - self.theta)

    def angular_command(self, error: float) -> float:
        return clamp(self.k_angular * error, self.max_angular)


@dataclass
class StepResult:
    linear: float
    angular: float
    remaining_angle: float
    next_state: 'MotionState'


class MotionState(ABC):
    name = 'BASE'

    @abstractmethod
    def step(self, ctx: ControlContext) -> StepResult:
        """Compute the command for this control cycle and the next state."""

    @property
    def done(self) -> bool:
        return False


class RotateToTarget(MotionState):
    """Turn in place until the turtle faces the target point."""

    name = 'ROTATE_TO_TARGET'

    def step(self, ctx):
        if ctx.distance_error <= ctx.position_tolerance:
            return RotateToYaw().step(ctx)
        error = ctx.heading_error
        if abs(error) <= ctx.heading_tolerance:
            return DriveToTarget().step(ctx)
        return StepResult(0.0, ctx.angular_command(error), error, self)


class DriveToTarget(MotionState):
    """Drive towards the target while correcting the heading."""

    name = 'DRIVE_TO_TARGET'
    # Above this heading error (far from the goal) stop and turn in place again.
    REALIGN_ANGLE = 0.6
    REALIGN_MIN_DISTANCE = 0.3

    def step(self, ctx):
        dist = ctx.distance_error
        if dist <= ctx.position_tolerance:
            return RotateToYaw().step(ctx)
        error = ctx.heading_error
        if abs(error) > self.REALIGN_ANGLE and dist > self.REALIGN_MIN_DISTANCE:
            return StepResult(0.0, ctx.angular_command(error), error, RotateToTarget())
        # Slow down when not facing the target so the path stays straight.
        linear = min(ctx.k_linear * dist, ctx.max_linear) * max(0.0, math.cos(error))
        return StepResult(linear, ctx.angular_command(error), error, self)


class RotateToYaw(MotionState):
    """Turn in place to the requested final heading."""

    name = 'ROTATE_TO_YAW'

    def step(self, ctx):
        error = ctx.yaw_error
        if abs(error) <= ctx.yaw_tolerance:
            return StepResult(0.0, 0.0, error, Reached())
        return StepResult(0.0, ctx.angular_command(error), error, self)


class Reached(MotionState):
    name = 'REACHED'

    def step(self, ctx):
        return StepResult(0.0, 0.0, ctx.yaw_error, self)

    @property
    def done(self) -> bool:
        return True
