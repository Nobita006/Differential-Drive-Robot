"""
Shared fixtures for lawn_mower_bot tests.
All tests run without a live ROS runtime — we mock what we need.
"""
import pytest
import math
import numpy as np


# ─── Constants matching the URDF / configs ──────────────────────────
WHEEL_RADIUS = 0.07
WHEEL_SEPARATION = 0.36
WHEEL_WIDTH = 0.04

EXPECTED_LINKS = [
    'base_link', 'base_footprint', 'left_wheel', 'right_wheel',
    'camera_link', 'lidar_link', 'imu_link', 'caster_front_wheel',
]

EXPECTED_JOINTS = {
    'base_footprint_joint': 'fixed',
    'left_wheel_joint': 'continuous',
    'right_wheel_joint': 'continuous',
    'camera_joint': 'fixed',
    'lidar_joint': 'fixed',
    'imu_joint': 'fixed',
    'caster_front_joint': 'fixed',
}


# ─── Fake map info (mimics nav_msgs/msg/MapMetaData) ────────────────
class FakeMapInfo:
    """Lightweight stand-in for OccupancyGrid.info."""
    def __init__(self, resolution=0.05, width=200, height=200,
                 origin_x=-5.0, origin_y=-5.0):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = type('O', (), {
            'position': type('P', (), {
                'x': origin_x, 'y': origin_y, 'z': 0.0
            })()
        })()


@pytest.fixture
def simple_map_info():
    """A 200×200, 0.05 m/cell grid centered near the origin."""
    return FakeMapInfo()


@pytest.fixture
def free_grid():
    """A 200×200 all-free occupancy grid (value 0)."""
    return np.zeros((200, 200), dtype=np.int8)


@pytest.fixture
def partially_occupied_grid():
    """A 200×200 grid with a 20-cell-wide obstacle band in the middle."""
    grid = np.zeros((200, 200), dtype=np.int8)
    grid[90:110, :] = 100  # occupied band
    return grid


@pytest.fixture
def green_image():
    """A 640×480 solid-green BGR image (pure grass)."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :] = (0, 180, 0)  # BGR green
    return img


@pytest.fixture
def red_image():
    """A 640×480 solid-red BGR image (all obstacles)."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :] = (0, 0, 200)  # BGR red
    return img
