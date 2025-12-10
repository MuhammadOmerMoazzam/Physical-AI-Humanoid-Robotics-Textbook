"""Utility functions for humanoid control."""
import math


def clamp(value, min_value, max_value):
    """Clamp a value between min and max."""
    return max(min_value, min(value, max_value))


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle