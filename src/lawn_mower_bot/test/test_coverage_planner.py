"""
Tests for CoveragePlanner logic — coordinate conversion, free-run
detection, BCD waypoint generation, and exclusion zones.

Runs without a live ROS graph by constructing the planner object
directly and injecting mock attributes.
"""
import sys
import os
import math
import pytest
import numpy as np
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

from conftest import FakeMapInfo  # noqa: E402


# ─────────────────────────────────────────────────────────────────────
#  Minimal planner-like object with just the methods we need to test
# ─────────────────────────────────────────────────────────────────────
class _PlannerProxy:
    """Extracted methods from CoveragePlanner for isolated testing."""

    def __init__(self, map_info=None):
        self.map_info = map_info
        self.lane_width = 1.0
        self.robot_radius = 0.15
        self.safety_margin = 0.5
        self.exclusion_zones = []
        self.mowed_cells = set()

    # --- imported methods (copy the exact logic) ---

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        wx = gx * self.map_info.resolution + self.map_info.origin.position.x + \
             self.map_info.resolution / 2.0
        wy = gy * self.map_info.resolution + self.map_info.origin.position.y + \
             self.map_info.resolution / 2.0
        return wx, wy

    def _is_in_exclusion_zone(self, wx, wy):
        for ez in self.exclusion_zones:
            cx = ez.get('center_x', 0)
            cy = ez.get('center_y', 0)
            r = ez.get('radius', 0) + self.safety_margin
            if math.sqrt((wx - cx)**2 + (wy - cy)**2) < r:
                return True
        return False

    def _find_free_runs(self, lane_free):
        runs = []
        min_run = max(3, int(self.robot_radius * 2 / self.map_info.resolution))
        in_run = False
        start = 0
        for i in range(len(lane_free)):
            if lane_free[i] and not in_run:
                start = i
                in_run = True
            elif not lane_free[i] and in_run:
                if i - start >= min_run:
                    runs.append((start, i - 1))
                in_run = False
        if in_run and len(lane_free) - start >= min_run:
            runs.append((start, len(lane_free) - 1))
        return runs

    def _boustrophedon_on_free_space(self, free_mask):
        height, width = free_mask.shape
        resolution = self.map_info.resolution
        lane_cells = max(1, int(self.lane_width / resolution))
        waypoints = []
        direction = 1
        y = 0
        while y < height:
            y_end = min(y + lane_cells, height)
            lane_free = np.any(free_mask[y:y_end, :], axis=0)
            runs = self._find_free_runs(lane_free)
            if runs:
                cy = (y + min(y + lane_cells - 1, height - 1)) / 2
                _, wy = self._grid_to_world(0, int(cy))
                for run_start, run_end in runs:
                    wx_start, _ = self._grid_to_world(run_start, int(cy))
                    wx_end, _ = self._grid_to_world(run_end, int(cy))
                    if direction == 1:
                        yaw = 0.0
                        waypoints.append({"x": wx_start, "y": wy, "yaw": yaw})
                        waypoints.append({"x": wx_end, "y": wy, "yaw": yaw})
                    else:
                        yaw = math.pi
                        waypoints.append({"x": wx_end, "y": wy, "yaw": yaw})
                        waypoints.append({"x": wx_start, "y": wy, "yaw": yaw})
                direction *= -1
            y += lane_cells
        return waypoints

    def _remove_mowed_waypoints(self, waypoints):
        if not self.mowed_cells:
            return waypoints
        filtered = []
        for wp in waypoints:
            gx, gy = self._world_to_grid(wp['x'], wp['y'])
            if (gx, gy) not in self.mowed_cells:
                filtered.append(wp)
        return filtered


# ─────────────────────────────────────────────────────────────────────
#  Test coordinate conversion
# ─────────────────────────────────────────────────────────────────────
class TestCoordinateConversion:
    def test_world_to_grid_origin(self, simple_map_info):
        p = _PlannerProxy(simple_map_info)
        gx, gy = p._world_to_grid(-5.0, -5.0)
        assert gx == 0
        assert gy == 0

    def test_grid_to_world_origin(self, simple_map_info):
        p = _PlannerProxy(simple_map_info)
        wx, wy = p._grid_to_world(0, 0)
        assert abs(wx - (-5.0 + 0.025)) < 1e-9
        assert abs(wy - (-5.0 + 0.025)) < 1e-9

    def test_roundtrip(self, simple_map_info):
        """world → grid → world should land near the original (within 1 cell)."""
        p = _PlannerProxy(simple_map_info)
        wx, wy = 2.5, -1.3
        gx, gy = p._world_to_grid(wx, wy)
        wx2, wy2 = p._grid_to_world(gx, gy)
        assert abs(wx2 - wx) < simple_map_info.resolution
        assert abs(wy2 - wy) < simple_map_info.resolution


# ─────────────────────────────────────────────────────────────────────
#  Test _find_free_runs
# ─────────────────────────────────────────────────────────────────────
class TestFindFreeRuns:
    def _runs(self, arr):
        p = _PlannerProxy(FakeMapInfo())
        return p._find_free_runs(np.array(arr, dtype=bool))

    def test_all_free(self):
        runs = self._runs([True] * 50)
        assert len(runs) == 1
        assert runs[0] == (0, 49)

    def test_all_blocked(self):
        runs = self._runs([False] * 50)
        assert len(runs) == 0

    def test_gap_in_middle(self):
        arr = [True] * 20 + [False] * 10 + [True] * 20
        runs = self._runs(arr)
        assert len(runs) == 2

    def test_short_runs_filtered(self):
        """Runs shorter than min_run should be filtered out."""
        arr = [True, True, False] + [True] * 30  # first run is only 2 cells
        runs = self._runs(arr)
        # Only the 30-cell run should survive (min_run ≈ 6 with default params)
        assert len(runs) == 1


# ─────────────────────────────────────────────────────────────────────
#  Test exclusion zones
# ─────────────────────────────────────────────────────────────────────
class TestExclusionZone:
    def test_inside_zone(self):
        p = _PlannerProxy()
        p.exclusion_zones = [{'center_x': 3.0, 'center_y': 3.0, 'radius': 1.0}]
        assert p._is_in_exclusion_zone(3.0, 3.0) is True

    def test_outside_zone(self):
        p = _PlannerProxy()
        p.exclusion_zones = [{'center_x': 3.0, 'center_y': 3.0, 'radius': 1.0}]
        assert p._is_in_exclusion_zone(0.0, 0.0) is False

    def test_on_boundary_with_safety(self):
        """A point at exactly radius distance should still be excluded
        because of `safety_margin`."""
        p = _PlannerProxy()
        p.exclusion_zones = [{'center_x': 0.0, 'center_y': 0.0, 'radius': 1.0}]
        # radius + safety_margin = 1.5, so at dist=1.2 it should be inside
        assert p._is_in_exclusion_zone(1.2, 0.0) is True


# ─────────────────────────────────────────────────────────────────────
#  Test BCD waypoint generation
# ─────────────────────────────────────────────────────────────────────
class TestBoustrophedon:
    def test_generates_waypoints_on_free_grid(self, simple_map_info, free_grid):
        p = _PlannerProxy(simple_map_info)
        free_mask = (free_grid == 0)
        waypoints = p._boustrophedon_on_free_space(free_mask)
        assert len(waypoints) > 0

    def test_alternating_direction(self, simple_map_info, free_grid):
        p = _PlannerProxy(simple_map_info)
        free_mask = (free_grid == 0)
        waypoints = p._boustrophedon_on_free_space(free_mask)
        # First lane should be east (yaw=0), second west (yaw=π)
        if len(waypoints) >= 4:
            assert waypoints[0]['yaw'] == 0.0
            assert abs(waypoints[2]['yaw'] - math.pi) < 1e-6

    def test_no_waypoints_on_occupied_grid(self, simple_map_info):
        p = _PlannerProxy(simple_map_info)
        occupied = np.full((200, 200), 100, dtype=np.int8)
        free_mask = (occupied == 0)
        waypoints = p._boustrophedon_on_free_space(free_mask)
        assert len(waypoints) == 0


# ─────────────────────────────────────────────────────────────────────
#  Test mowed-waypoint removal
# ─────────────────────────────────────────────────────────────────────
class TestRemoveMowedWaypoints:
    def test_removes_mowed(self, simple_map_info):
        p = _PlannerProxy(simple_map_info)
        wps = [{'x': -4.9, 'y': -4.9, 'yaw': 0.0},
               {'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
        # Mark the first waypoint's grid cell
        gx, gy = p._world_to_grid(-4.9, -4.9)
        p.mowed_cells.add((gx, gy))
        filtered = p._remove_mowed_waypoints(wps)
        assert len(filtered) == 1
        assert filtered[0]['x'] == 0.0

    def test_no_mowed_returns_all(self, simple_map_info):
        p = _PlannerProxy(simple_map_info)
        wps = [{'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
        filtered = p._remove_mowed_waypoints(wps)
        assert len(filtered) == 1
