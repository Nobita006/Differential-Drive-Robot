"""
Tests for boustrophedon waypoint generation — verifies the
mow_lawn.py coverage pattern covers the expected area.

Replicates the logic inline to avoid importing ROS-dependent modules.
"""
import math
import pytest


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class FakePose:
    def __init__(self, x, y, z, orientation):
        self.position = type('P', (), {'x': x, 'y': y, 'z': z})()
        self.orientation = orientation


class FakePoseStamped:
    def __init__(self, frame_id, x, y, z, orientation):
        self.header = type('H', (), {'frame_id': frame_id})()
        self.pose = FakePose(x, y, z, orientation)


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - \
         math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + \
         math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - \
         math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + \
         math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def make_pose(x, y, yaw, frame='map'):
    orientation = get_quaternion_from_euler(0.0, 0.0, yaw)
    return FakePoseStamped(frame, float(x), float(y), 0.0, orientation)


def generate_coverage_waypoints():
    """Exact copy of the logic from mow_lawn.py."""
    waypoints = []
    x_min, x_max = -7.0, 7.0
    y_min, y_max = -7.0, 7.0
    lane_width = 1.0
    y = y_min
    direction = 1
    while y <= y_max:
        if direction == 1:
            waypoints.append(make_pose(x_min, y, 0.0))
            waypoints.append(make_pose(x_max, y, 0.0))
        else:
            waypoints.append(make_pose(x_max, y, math.pi))
            waypoints.append(make_pose(x_min, y, math.pi))
        direction *= -1
        y += lane_width
    return waypoints


class TestGenerateCoverageWaypoints:
    @pytest.fixture
    def waypoints(self):
        return generate_coverage_waypoints()

    def test_waypoint_count(self, waypoints):
        """y from -7 to 7 in steps of 1 → 15 lanes × 2 endpoints = 30."""
        assert len(waypoints) == 30

    def test_first_lane_faces_east(self, waypoints):
        q = waypoints[0].pose.orientation
        expected = get_quaternion_from_euler(0.0, 0.0, 0.0)
        assert abs(q.z - expected.z) < 1e-6
        assert abs(q.w - expected.w) < 1e-6

    def test_second_lane_faces_west(self, waypoints):
        q = waypoints[2].pose.orientation
        expected = get_quaternion_from_euler(0.0, 0.0, math.pi)
        assert abs(q.z - expected.z) < 1e-6
        assert abs(q.w - expected.w) < 1e-6

    def test_y_range(self, waypoints):
        ys = sorted(set(wp.pose.position.y for wp in waypoints))
        assert abs(ys[0] - (-7.0)) < 1e-6
        assert abs(ys[-1] - 7.0) < 1e-6

    def test_x_range(self, waypoints):
        xs = set(wp.pose.position.x for wp in waypoints)
        assert -7.0 in xs
        assert 7.0 in xs

    def test_all_poses_in_map_frame(self, waypoints):
        for wp in waypoints:
            assert wp.header.frame_id == 'map'

    def test_z_is_zero(self, waypoints):
        for wp in waypoints:
            assert wp.pose.position.z == 0.0

    def test_lane_spacing(self, waypoints):
        """Consecutive lanes should be 1m apart in Y."""
        ys = sorted(set(wp.pose.position.y for wp in waypoints))
        for i in range(1, len(ys)):
            assert abs(ys[i] - ys[i-1] - 1.0) < 1e-6
