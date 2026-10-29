"""Small, ROS-free geometry helpers."""

import math


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, limit]."""
    return max(-limit, min(limit, value))
