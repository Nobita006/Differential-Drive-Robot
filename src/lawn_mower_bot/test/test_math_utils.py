"""
Tests for the quaternion / pose utility functions shared across
mow_lawn.py, simple_move.py, and gemini_mow_executor.py.

These functions are duplicated across scripts, so we test the logic
directly without importing the ROS-dependent modules.
"""
import math
import pytest

# ─── Replicate the pure math functions from the scripts ─────────────

class Quaternion:
    """Lightweight quaternion to avoid importing geometry_msgs."""
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def get_quaternion_from_euler(roll, pitch, yaw):
    """Exact copy from mow_lawn.py / simple_move.py."""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class TestGetQuaternionFromEuler:
    """Verify euler-to-quaternion conversion."""

    def test_identity(self):
        """Zero angles should give identity quaternion (0,0,0,1)."""
        q = get_quaternion_from_euler(0.0, 0.0, 0.0)
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z) < 1e-9
        assert abs(q.w - 1.0) < 1e-9

    def test_yaw_90_degrees(self):
        """90° yaw → qz ≈ sin(π/4), qw ≈ cos(π/4)."""
        q = get_quaternion_from_euler(0.0, 0.0, math.pi / 2)
        assert abs(q.z - math.sin(math.pi / 4)) < 1e-6
        assert abs(q.w - math.cos(math.pi / 4)) < 1e-6
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9

    def test_yaw_180_degrees(self):
        """180° yaw → qz ≈ 1, qw ≈ 0."""
        q = get_quaternion_from_euler(0.0, 0.0, math.pi)
        assert abs(q.z - 1.0) < 1e-6
        assert abs(q.w) < 1e-6

    def test_unit_quaternion(self):
        """Quaternion must have unit magnitude for any input."""
        for yaw in [0, 0.5, 1.0, math.pi, -math.pi / 3, 2 * math.pi]:
            q = get_quaternion_from_euler(0.0, 0.0, yaw)
            mag = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            assert abs(mag - 1.0) < 1e-9, f"Non-unit quaternion for yaw={yaw}"

    def test_negative_yaw(self):
        """Negative yaw should mirror positive yaw in qz sign."""
        q_pos = get_quaternion_from_euler(0.0, 0.0, math.pi / 4)
        q_neg = get_quaternion_from_euler(0.0, 0.0, -math.pi / 4)
        assert abs(q_pos.z + q_neg.z) < 1e-9  # opposite signs
        assert abs(q_pos.w - q_neg.w) < 1e-9  # same w

    def test_roll_only(self):
        """Roll=90° with pitch=yaw=0 should only affect x and w."""
        q = get_quaternion_from_euler(math.pi / 2, 0.0, 0.0)
        assert abs(q.x - math.sin(math.pi / 4)) < 1e-6
        assert abs(q.w - math.cos(math.pi / 4)) < 1e-6
        assert abs(q.y) < 1e-9
        assert abs(q.z) < 1e-9

    def test_pitch_only(self):
        """Pitch=90° with roll=yaw=0 should only affect y and w."""
        q = get_quaternion_from_euler(0.0, math.pi / 2, 0.0)
        assert abs(q.y - math.sin(math.pi / 4)) < 1e-6
        assert abs(q.w - math.cos(math.pi / 4)) < 1e-6
        assert abs(q.x) < 1e-9
        assert abs(q.z) < 1e-9
