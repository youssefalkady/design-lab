"""Simulate the goToPose state machine with a unicycle model (no ROS needed)."""

import math

import pytest

from turtle_controller.geometry import normalize_angle
from turtle_controller.motion_states import ControlContext, RotateToTarget

DT = 0.02


def simulate(start, goal, max_linear=2.0, max_angular=2.0, steps=5000):
    x, y, theta = start
    state = RotateToTarget()
    max_v = max_w = 0.0
    for _ in range(steps):
        ctx = ControlContext(
            x=x, y=y, theta=theta, goal_x=goal[0], goal_y=goal[1], goal_yaw=goal[2],
            max_linear=max_linear, max_angular=max_angular, k_linear=1.5,
            k_angular=4.0, position_tolerance=0.05, yaw_tolerance=0.01,
            heading_tolerance=0.05)
        step = state.step(ctx)
        state = step.next_state
        if state.done:
            return x, y, theta, max_v, max_w
        max_v, max_w = max(max_v, abs(step.linear)), max(max_w, abs(step.angular))
        theta = normalize_angle(theta + step.angular * DT)
        x += step.linear * math.cos(theta) * DT
        y += step.linear * math.sin(theta) * DT
    pytest.fail('goal not reached')


def test_normalize_angle():
    assert normalize_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert normalize_angle(0.5) == pytest.approx(0.5)


@pytest.mark.parametrize('goal', [
    (9.0, 5.5, 0.0), (2.0, 5.5, math.pi), (5.5, 9.0, 1.0),
    (5.5, 1.0, -2.0), (1.0, 1.0, 3.0), (5.55, 5.55, -math.pi / 2),
])
def test_reaches_goal_in_all_directions(goal):
    x, y, theta, _, _ = simulate((5.544, 5.544, 0.0), goal)
    assert math.hypot(goal[0] - x, goal[1] - y) < 0.1
    assert abs(normalize_angle(goal[2] - theta)) < 0.05


def test_respects_speed_limits():
    *_, max_v, max_w = simulate((1.0, 1.0, 0.0), (9.0, 9.0, 2.0),
                                max_linear=0.5, max_angular=0.7)
    assert max_v <= 0.5 + 1e-9
    assert max_w <= 0.7 + 1e-9
