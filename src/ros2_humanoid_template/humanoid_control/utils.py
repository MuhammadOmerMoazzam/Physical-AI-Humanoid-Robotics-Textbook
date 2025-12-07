# humanoid_control/utils.py
# Utility functions for humanoid control

def normalize_joint_limits(position, min_limit=-3.14, max_limit=3.14):
    """Normalize joint positions to valid range"""
    return max(min_limit, min(max_limit, position))

def calculate_interpolation(start_pos, end_pos, t):
    """Linearly interpolate between two positions (t from 0 to 1)"""
    return start_pos + t * (end_pos - start_pos)