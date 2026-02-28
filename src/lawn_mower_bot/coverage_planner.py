#!/usr/bin/env python3
"""
Coverage Path Planner — Boustrophedon Cell Decomposition on Costmap

Reads the occupancy grid from SLAM and generates optimal coverage waypoints
for any lawn shape. Handles exclusion zones, tracks mowed area, and supports
re-planning when areas are missed.

Subscribes:
    /map                (OccupancyGrid)  — from SLAM toolbox
    /gemini/mow_plan    (String)         — exclusion zones from AI planner
    /coverage/replan    (String)         — re-plan request with missed waypoints

Publishes:
    /coverage/waypoints (String)  — JSON list of coverage waypoints
    /coverage/status    (String)  — JSON coverage statistics
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import json
import math


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        # ---------- Parameters ----------
        self.declare_parameter('lane_width', 1.0)
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('min_map_coverage', 0.3)  # min free cells before planning

        self.lane_width = self.get_parameter('lane_width').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.safety_margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        self.min_map_coverage = self.get_parameter('min_map_coverage').get_parameter_value().double_value

        # ---------- State ----------
        self.latest_map = None
        self.map_info = None
        self.exclusion_zones = []
        self.mowed_cells = set()  # (grid_x, grid_y) of mowed cells
        self.plan_ready = False

        # ---------- Subscribers ----------
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.create_subscription(String, '/gemini/mow_plan', self._plan_cb, 10)
        self.create_subscription(String, '/coverage/replan', self._replan_cb, 10)
        self.create_subscription(String, '/coverage/mark_mowed', self._mark_mowed_cb, 10)

        # ---------- Publishers ----------
        self.waypoints_pub = self.create_publisher(String, '/coverage/waypoints', 10)
        self.status_pub = self.create_publisher(String, '/coverage/status', 10)

        # ---------- Timer for status updates ----------
        self.create_timer(5.0, self._publish_status)

        self.get_logger().info("CoveragePlanner ready — waiting for map from SLAM")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _map_cb(self, msg: OccupancyGrid):
        """Store the latest occupancy grid from SLAM."""
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.latest_map = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.get_logger().debug(
            f"Map received: {width}x{height}, resolution={msg.info.resolution}m"
        )

    def _plan_cb(self, msg):
        """Extract exclusion zones from the AI plan."""
        try:
            plan = json.loads(msg.data)
            self.exclusion_zones = plan.get('exclusion_zones', [])
            lane = plan.get('lane_width', None)
            if lane:
                self.lane_width = float(lane)
            self.get_logger().info(
                f"Received AI plan: {len(self.exclusion_zones)} exclusion zones, "
                f"lane_width={self.lane_width}m"
            )
            # Trigger planning with the new exclusion zones
            self._generate_coverage()
        except Exception as e:
            self.get_logger().error(f"Failed to parse AI plan: {e}")

    def _replan_cb(self, msg):
        """Re-plan coverage, optionally for missed waypoints only."""
        try:
            data = json.loads(msg.data)
            missed = data.get('missed_waypoints', [])
            if missed:
                self.get_logger().info(
                    f"Re-plan requested for {len(missed)} missed waypoints"
                )
            else:
                self.get_logger().info("Full re-plan requested")
                self.mowed_cells.clear()
            self._generate_coverage()
        except Exception:
            # Simple text command
            self.get_logger().info("Re-plan requested")
            self._generate_coverage()

    def _mark_mowed_cb(self, msg):
        """Mark an area as mowed (from executor)."""
        try:
            data = json.loads(msg.data)
            x = data['x']
            y = data['y']
            radius = data.get('radius', 0.5)
            self._mark_mowed_area(x, y, radius)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Coordinate conversions
    # ------------------------------------------------------------------ #
    def _world_to_grid(self, wx, wy):
        """Convert world coordinates to grid indices."""
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates (cell center)."""
        wx = gx * self.map_info.resolution + self.map_info.origin.position.x + \
             self.map_info.resolution / 2.0
        wy = gy * self.map_info.resolution + self.map_info.origin.position.y + \
             self.map_info.resolution / 2.0
        return wx, wy

    def _is_in_exclusion_zone(self, wx, wy):
        """Check if a world coordinate is inside any exclusion zone."""
        for ez in self.exclusion_zones:
            cx = ez.get('center_x', 0)
            cy = ez.get('center_y', 0)
            r = ez.get('radius', 0) + self.safety_margin
            if math.sqrt((wx - cx)**2 + (wy - cy)**2) < r:
                return True
        return False

    def _mark_mowed_area(self, wx, wy, radius):
        """Mark grid cells around a world position as mowed."""
        if self.map_info is None:
            return
        r_cells = int(radius / self.map_info.resolution)
        gx, gy = self._world_to_grid(wx, wy)
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx*dx + dy*dy <= r_cells*r_cells:
                    self.mowed_cells.add((gx + dx, gy + dy))

    # ------------------------------------------------------------------ #
    # Boustrophedon Cell Decomposition
    # ------------------------------------------------------------------ #
    def _get_free_space_mask(self):
        """
        Create a binary mask of free space from the occupancy grid.
        Also inflates obstacles by robot_radius for safety.
        """
        grid = self.latest_map
        height, width = grid.shape

        # Free = 0, Unknown = -1, Occupied >= 50
        # Treat unknown as free for coverage (we want to try to reach it)
        free_mask = np.zeros((height, width), dtype=bool)
        free_mask[grid == 0] = True        # definitely free
        free_mask[grid == -1] = False      # unknown — don't mow into unknown

        # Inflate obstacles by robot radius
        inflate_cells = int((self.robot_radius + 0.1) / self.map_info.resolution)
        if inflate_cells > 0:
            from scipy import ndimage
            obstacle_mask = (grid >= 50)
            inflated = ndimage.binary_dilation(
                obstacle_mask,
                iterations=inflate_cells
            )
            free_mask[inflated] = False

        # Mask exclusion zones
        for gy in range(height):
            for gx in range(width):
                wx, wy = self._grid_to_world(gx, gy)
                if self._is_in_exclusion_zone(wx, wy):
                    free_mask[gy, gx] = False

        return free_mask

    def _boustrophedon_on_free_space(self, free_mask):
        """
        Generate boustrophedon (back-and-forth) waypoints covering all free space.

        Works by scanning the free space in horizontal lanes, generating
        waypoints at lane transitions and endpoints. This naturally adapts
        to any lawn shape since it only generates waypoints within free cells.
        """
        height, width = free_mask.shape
        resolution = self.map_info.resolution
        lane_cells = max(1, int(self.lane_width / resolution))

        waypoints = []
        direction = 1  # 1 = left-to-right, -1 = right-to-left

        # Scan in horizontal lanes from bottom to top
        y = 0
        while y < height:
            # Find the x-range of free cells in this lane
            # Average over a few rows for stability
            y_end = min(y + lane_cells, height)
            lane_free = np.any(free_mask[y:y_end, :], axis=0)

            # Find contiguous runs of free space in this lane
            runs = self._find_free_runs(lane_free)

            if runs:
                # Center Y of this lane
                cy = (y + min(y + lane_cells - 1, height - 1)) / 2
                _, wy = self._grid_to_world(0, int(cy))

                for run_start, run_end in runs:
                    wx_start, _ = self._grid_to_world(run_start, int(cy))
                    wx_end, _ = self._grid_to_world(run_end, int(cy))

                    if direction == 1:
                        yaw = 0.0  # facing east
                        waypoints.append({"x": wx_start, "y": wy, "yaw": yaw})
                        waypoints.append({"x": wx_end, "y": wy, "yaw": yaw})
                    else:
                        yaw = math.pi  # facing west
                        waypoints.append({"x": wx_end, "y": wy, "yaw": yaw})
                        waypoints.append({"x": wx_start, "y": wy, "yaw": yaw})

                direction *= -1

            y += lane_cells

        return waypoints

    def _find_free_runs(self, lane_free):
        """
        Find contiguous runs of True values in a 1D boolean array.
        Returns list of (start_idx, end_idx) tuples.
        Skips very short runs (less than a few cells).
        """
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

        # Handle run that extends to the edge
        if in_run and len(lane_free) - start >= min_run:
            runs.append((start, len(lane_free) - 1))

        return runs

    def _remove_mowed_waypoints(self, waypoints):
        """Remove waypoints in already-mowed areas."""
        if not self.mowed_cells:
            return waypoints

        filtered = []
        for wp in waypoints:
            gx, gy = self._world_to_grid(wp['x'], wp['y'])
            if (gx, gy) not in self.mowed_cells:
                filtered.append(wp)

        return filtered

    # ------------------------------------------------------------------ #
    # Main planning logic
    # ------------------------------------------------------------------ #
    def _generate_coverage(self):
        """Generate coverage waypoints from the current map."""
        if self.latest_map is None or self.map_info is None:
            self.get_logger().warn("No map available yet — cannot plan coverage")
            return

        height, width = self.latest_map.shape
        total_cells = height * width
        free_cells = np.sum(self.latest_map == 0)
        free_pct = free_cells / total_cells if total_cells > 0 else 0

        if free_pct < self.min_map_coverage:
            self.get_logger().warn(
                f"Map only {free_pct:.0%} free — waiting for more SLAM data "
                f"(need {self.min_map_coverage:.0%})"
            )
            return

        self.get_logger().info(
            f"Planning coverage on {width}x{height} map "
            f"({free_pct:.0%} free, {len(self.exclusion_zones)} exclusions)"
        )

        # Step 1: Get free space mask
        try:
            free_mask = self._get_free_space_mask()
        except ImportError:
            # scipy not available — use simple approach without inflation
            self.get_logger().warn("scipy not available — skipping obstacle inflation")
            free_mask = (self.latest_map == 0)
            for gy in range(height):
                for gx in range(width):
                    wx, wy = self._grid_to_world(gx, gy)
                    if self._is_in_exclusion_zone(wx, wy):
                        free_mask[gy, gx] = False

        # Step 2: BCD coverage
        waypoints = self._boustrophedon_on_free_space(free_mask)

        # Step 3: Remove already-mowed areas
        waypoints = self._remove_mowed_waypoints(waypoints)

        if len(waypoints) == 0:
            self.get_logger().info("No unmowed waypoints left — coverage complete!")
            self._publish_coverage_complete()
            return

        self.get_logger().info(f"Generated {len(waypoints)} coverage waypoints")

        # Step 4: Publish
        msg = String()
        msg.data = json.dumps({
            "waypoints": waypoints,
            "total_waypoints": len(waypoints),
            "lane_width": self.lane_width,
            "exclusion_zones": self.exclusion_zones,
            "free_space_pct": round(free_pct * 100, 1),
            "source": "coverage_planner_bcd",
        })
        self.waypoints_pub.publish(msg)
        self.plan_ready = True

    # ------------------------------------------------------------------ #
    # Status publishing
    # ------------------------------------------------------------------ #
    def _publish_status(self):
        """Publish coverage statistics."""
        if self.latest_map is None:
            return

        total_free = int(np.sum(self.latest_map == 0))
        mowed = len(self.mowed_cells)
        coverage_pct = (mowed / total_free * 100) if total_free > 0 else 0

        status = {
            "coverage_percent": round(coverage_pct, 1),
            "mowed_cells": mowed,
            "total_free_cells": total_free,
            "map_size": f"{self.latest_map.shape[1]}x{self.latest_map.shape[0]}",
            "plan_ready": self.plan_ready,
            "exclusion_zones": len(self.exclusion_zones),
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _publish_coverage_complete(self):
        """Publish a completion signal."""
        msg = String()
        msg.data = json.dumps({
            "waypoints": [],
            "total_waypoints": 0,
            "complete": True,
            "source": "coverage_planner_bcd",
        })
        self.waypoints_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
