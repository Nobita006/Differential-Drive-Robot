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
from geometry_msgs.msg import Point
import numpy as np
import scipy.ndimage as ndi
import json
import math


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        # ---------- Parameters ----------
        self.declare_parameter('lane_width', 0.5)
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('min_map_coverage', 0.01)  # min free cells before planning

        self.lane_width = self.get_parameter('lane_width').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.safety_margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        self.min_map_coverage = self.get_parameter('min_map_coverage').get_parameter_value().double_value

        # ---------- State ----------
        self.latest_map = None
        self.map_info = None
        self.exclusion_zones = []
        self.mowed_cells = set()  # (grid_x, grid_y) of mowed cells
        self.failed_frontiers = []
        self.plan_ready = False

        # ---------- Subscribers ----------
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.create_subscription(String, '/gemini/mow_plan', self._plan_cb, 10)
        self.create_subscription(String, '/coverage/replan', self._replan_cb, 10)
        self.create_subscription(String, '/coverage/mark_mowed', self._mark_mowed_cb, 10)

        # ---------- Publishers ----------
        self.waypoints_pub = self.create_publisher(String, '/coverage/waypoints', 10)
        self.status_pub = self.create_publisher(String, '/coverage/status', 10)
        self.area_pub = self.create_publisher(OccupancyGrid, '/mowed_area', 10)

        # ---------- Timer for status updates ----------
        self.create_timer(5.0, self._publish_status)
        self.create_timer(2.0, self._publish_area_map)

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
            command = data.get('command', '')
            
            if command == 'get_frontier':
                failed_frontier = data.get('failed_frontier')
                if failed_frontier:
                    self.failed_frontiers.append(failed_frontier)
                self._handle_frontier_request()
                return
                
            missed = data.get('missed_waypoints', [])
            if missed:
                self.get_logger().info(
                    f"Re-plan requested for {len(missed)} missed waypoints"
                )
            else:
                self.get_logger().info("Full re-plan requested")
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

        # Mask exclusion zones (vectorized — no Python loops)
        if self.exclusion_zones:
            gy_coords, gx_coords = np.mgrid[0:height, 0:width]
            wx_coords = gx_coords * self.map_info.resolution + self.map_info.origin.position.x + self.map_info.resolution / 2.0
            wy_coords = gy_coords * self.map_info.resolution + self.map_info.origin.position.y + self.map_info.resolution / 2.0
            for ez in self.exclusion_zones:
                cx = ez.get('center_x', 0)
                cy = ez.get('center_y', 0)
                r = ez.get('radius', 0) + self.safety_margin
                dist_sq = (wx_coords - cx)**2 + (wy_coords - cy)**2
                free_mask[dist_sq < r*r] = False

        return free_mask

    def _perimeter_on_free_space(self, free_mask):
        """Generate waypoints tracing the outer contour of the free space."""
        waypoints = []
        try:
            import cv2
        except ImportError:
            self.get_logger().warn("OpenCV not available — skipping perimeter pass")
            return waypoints
            
        mask_uint8 = (free_mask * 255).astype(np.uint8)
        
        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return waypoints

        largest_contour = max(contours, key=cv2.contourArea)
        
        # Sample points along the contour (roughly every 0.5 meters)
        step = max(1, int(0.5 / self.map_info.resolution))
        if step >= len(largest_contour):
            step = 1
            
        pts = largest_contour[::step]
        if len(pts) < 2:
            return waypoints
            
        for i in range(len(pts)):
            pt1 = pts[i][0]
            pt2 = pts[(i+1)%len(pts)][0]
            gx, gy = float(pt1[0]), float(pt1[1])
            wx, wy = self._grid_to_world(gx, gy)
            
            gx2, gy2 = float(pt2[0]), float(pt2[1])
            wx2, wy2 = self._grid_to_world(gx2, gy2)
            yaw = math.atan2(wy2 - wy, wx2 - wx)
            
            waypoints.append({"x": float(wx), "y": float(wy), "yaw": float(yaw)})
            
        return waypoints

    def _boustrophedon_on_free_space(self, free_mask, start_pos=None):
        """
        Generate boustrophedon (back-and-forth) waypoints covering all free space.
        
        Uses clean alternating-direction rows: sweep left→right on row 1,
        right→left on row 2, etc. This produces the straight-line patterns
        that real mowers use.
        """
        height, width = free_mask.shape
        resolution = self.map_info.resolution
        lane_cells = max(1, int(self.lane_width / resolution))

        waypoints = []
        direction = 1  # 1 = left-to-right, -1 = right-to-left

        y = 0
        while y < height:
            y_end = min(y + lane_cells, height)
            lane_free = np.any(free_mask[y:y_end, :], axis=0)
            runs = self._find_free_runs(lane_free)
            
            if runs:
                cy = (y + min(y + lane_cells - 1, height - 1)) / 2
                _, wy = self._grid_to_world(0, int(cy))
                
                # Process runs in the current sweep direction
                ordered_runs = list(runs) if direction == 1 else list(reversed(runs))
                    
                for run_start, run_end in ordered_runs:
                    wx_start, _ = self._grid_to_world(run_start, int(cy))
                    wx_end, _ = self._grid_to_world(run_end, int(cy))
                    
                    if direction == 1:
                        yaw = 0.0  # facing right
                        waypoints.append({"x": float(wx_start), "y": float(wy), "yaw": yaw})
                        waypoints.append({"x": float(wx_end), "y": float(wy), "yaw": yaw})
                    else:
                        yaw = math.pi  # facing left
                        waypoints.append({"x": float(wx_end), "y": float(wy), "yaw": yaw})
                        waypoints.append({"x": float(wx_start), "y": float(wy), "yaw": yaw})
                        
                # Flip direction after each row
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
        """Remove sweep PAIRS where both endpoints are already mowed.
        
        Waypoints come in pairs (sweep_start, sweep_end). Removing individual
        points breaks the pairing and creates erratic diagonal jumps.
        """
        if not self.mowed_cells:
            return waypoints

        filtered = []
        i = 0
        while i < len(waypoints):
            if i + 1 < len(waypoints):
                wp_start = waypoints[i]
                wp_end = waypoints[i + 1]
                gx1, gy1 = self._world_to_grid(wp_start['x'], wp_start['y'])
                gx2, gy2 = self._world_to_grid(wp_end['x'], wp_end['y'])
                start_mowed = (gx1, gy1) in self.mowed_cells
                end_mowed = (gx2, gy2) in self.mowed_cells
                
                if start_mowed and end_mowed:
                    # Both endpoints already mowed — skip entire sweep
                    i += 2
                    continue
                    
                # Keep the pair intact
                filtered.append(wp_start)
                filtered.append(wp_end)
                i += 2
            else:
                # Odd waypoint at the end (shouldn't normally happen)
                gx, gy = self._world_to_grid(waypoints[i]['x'], waypoints[i]['y'])
                if (gx, gy) not in self.mowed_cells:
                    filtered.append(waypoints[i])
                i += 1

        return filtered

    # ------------------------------------------------------------------ #
    # Frontier Exploration logic
    # ------------------------------------------------------------------ #
    def _handle_frontier_request(self):
        """Find the largest frontier and publish as a single waypoint."""
        if self.latest_map is None or self.map_info is None:
            self.get_logger().warn("Map not available yet — can't search frontiers, telling executor to retry")
            msg = String()
            msg.data = json.dumps({
                "waypoints": [],
                "total_waypoints": 0,
                "complete": False,
                "source": "frontier_explorer"
            })
            self.waypoints_pub.publish(msg)
            return

        self.get_logger().info("Frontier request received. Searching map...")
        frontier_wp = self._find_frontier()
        
        msg = String()
        if frontier_wp:
            self.get_logger().info(f"Frontier found at {frontier_wp['x']:.2f}, {frontier_wp['y']:.2f}")
            msg.data = json.dumps({
                "waypoints": [frontier_wp],
                "total_waypoints": 1,
                "complete": False,
                "source": "frontier_explorer"
            })
        else:
            self.get_logger().info("NO FRONTIERS FOUND. Map is complete!")
            msg.data = json.dumps({
                "waypoints": [],
                "total_waypoints": 0,
                "complete": True,
                "source": "frontier_explorer"
            })
        self.waypoints_pub.publish(msg)
        
    def _find_frontier(self):
        """Find the center of the largest frontier using image morphology."""
        if self.latest_map is None:
            return None
            
        # 0 is free, -1 is unknown, 100 is occupied
        free_space = (self.latest_map == 0).astype(np.uint8)
        unknown_space = (self.latest_map == -1).astype(np.uint8)
        
        # Dilate free space to find where it touches unknown space
        kernel = np.ones((3,3), dtype=np.uint8)
        dilated_free = ndi.binary_dilation(free_space, structure=kernel).astype(np.uint8)
        
        # Frontier is the intersection of dilated free space and unknown space
        frontier_mask = (dilated_free & unknown_space)
        
        if not np.any(frontier_mask):
            return None
            
        # Group frontier pixels into distinct contiguous regions
        labeled_frontiers, num_features = ndi.label(frontier_mask)
        
        if num_features == 0:
            return None
            
        # Find the largest frontier
        sizes = ndi.sum(frontier_mask, labeled_frontiers, range(1, num_features + 1))
        
        # Filter out tiny frontiers (noise)
        min_frontier_size = int(1.0 / self.map_info.resolution) # 1 meter long frontier
        valid_frontiers = [i for i, size in enumerate(sizes) if size >= min_frontier_size]
        
        if not valid_frontiers:
            return None
            
        # Sort valid frontiers by size descending
        valid_frontiers.sort(key=lambda i: sizes[i], reverse=True)
        
        # Erode free space for safe navigation targets (done once for all frontiers)
        safe_cells = int((self.robot_radius + 0.1) / self.map_info.resolution)
        if safe_cells > 0:
            safe_free_space = ndi.binary_erosion(free_space, iterations=safe_cells).astype(int)
        else:
            safe_free_space = free_space
        
        height, width = safe_free_space.shape
        search_radius = safe_cells + 30  # ~1.5m search radius for safe pixels
        
        for idx in valid_frontiers:
            label = idx + 1
            cy, cx = ndi.center_of_mass(frontier_mask, labeled_frontiers, label)
            wx, wy = self._grid_to_world(int(cx), int(cy))
            
            # Check against failed frontiers (within 1.0m)
            is_failed = False
            for failed_fp in self.failed_frontiers:
                dist = math.hypot(wx - failed_fp['x'], wy - failed_fp['y'])
                if dist < 1.0:
                    is_failed = True
                    break
                    
            if is_failed:
                self.get_logger().info(f"Skipping frontier near {wx:.2f}, {wy:.2f} (previously failed)")
                continue
            
            # Find the nearest safe free pixel to this frontier's center of mass
            cxi, cyi = int(cx), int(cy)
            best_x, best_y = cxi, cyi
            min_dist = float('inf')
            found_safe_pixel = False
            
            for y in range(max(0, cyi - search_radius), min(height, cyi + search_radius + 1)):
                for x in range(max(0, cxi - search_radius), min(width, cxi + search_radius + 1)):
                    if safe_free_space[y, x] == 1:
                        dist = (x - cxi)**2 + (y - cyi)**2
                        if dist < min_dist:
                            min_dist = dist
                            best_x, best_y = x, y
                            found_safe_pixel = True
            
            if not found_safe_pixel:
                self.get_logger().info(f"Skipping frontier near {wx:.2f}, {wy:.2f} (no safe pixel nearby)")
                continue
            
            # Found a valid frontier with a safe navigation target
            wx, wy = self._grid_to_world(best_x, best_y)
            yaw = math.atan2(cyi - best_y, cxi - best_x)
            return {"x": wx, "y": wy, "yaw": yaw}
        
        # No valid frontiers with safe pixels found
        self.get_logger().warn("All frontiers exhausted (failed or no safe pixel). Exploration may be complete.")
        return None

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
            if self.exclusion_zones:
                gy_coords, gx_coords = np.mgrid[0:height, 0:width]
                wx_coords = gx_coords * self.map_info.resolution + self.map_info.origin.position.x + self.map_info.resolution / 2.0
                wy_coords = gy_coords * self.map_info.resolution + self.map_info.origin.position.y + self.map_info.resolution / 2.0
                for ez in self.exclusion_zones:
                    cx = ez.get('center_x', 0)
                    cy = ez.get('center_y', 0)
                    r = ez.get('radius', 0) + self.safety_margin
                    dist_sq = (wx_coords - cx)**2 + (wy_coords - cy)**2
                    free_mask[dist_sq < r*r] = False

        # Step 2: Coverage Generation (Perimeter + Boustrophedon)
        waypoints = self._perimeter_on_free_space(free_mask)
        
        # Erode the mask by lane_width so interior boustrophedon doesn't overlap edges
        try:
            from scipy import ndimage
            lane_cells = max(1, int(self.lane_width / self.map_info.resolution))
            inner_mask = ndimage.binary_erosion(free_mask, iterations=lane_cells)
        except ImportError:
            inner_mask = free_mask
            
        start_pos = waypoints[-1] if waypoints else None
        waypoints.extend(self._boustrophedon_on_free_space(inner_mask, start_pos))

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

    def _publish_area_map(self):
        """Publish an OccupancyGrid showing only the mowed area."""
        if self.latest_map is None or self.map_info is None:
            return

        height, width = self.latest_map.shape
        grid = np.full((height, width), -1, dtype=np.int8)

        # Mark free space as 0 (unmowed)
        grid[self.latest_map == 0] = 0

        # Mark mowed cells as 100 (vectorized)
        if self.mowed_cells:
            cells = np.array(list(self.mowed_cells))
            gx_arr = cells[:, 0]
            gy_arr = cells[:, 1]
            valid = (gx_arr >= 0) & (gx_arr < width) & (gy_arr >= 0) & (gy_arr < height)
            grid[gy_arr[valid], gx_arr[valid]] = 100

        area_msg = OccupancyGrid()
        area_msg.header.frame_id = 'map'
        area_msg.header.stamp = self.get_clock().now().to_msg()
        area_msg.info = self.map_info
        area_msg.data = grid.flatten().tolist()
        
        self.area_pub.publish(area_msg)


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
