#!/usr/bin/env python3
"""
Gemini AI Mow Executor — Demo-Ready

Executes autonomous mowing in three phases:
  Phase 1: Boundary Discovery — frontier exploration to let SLAM build the map
  Phase 2: Coverage Mowing — receives waypoints from coverage_planner, executes
  Phase 3: Return to Dock — navigates back to the starting position

Features:
  - Frontier exploration with retry logic
  - Stuck detection & automatic recovery (backup/spin)
  - Adaptive speed near obstacles
  - Return-to-dock after mowing
  - Rich RViz visualization (planned path, actual trail, deviation, mowed area)
  - Coverage progress tracking
  - Dynamic replanning during mowing
"""
import rclpy
import math
import time
import json
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, ColorRGBA, Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


def get_quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to a geometry_msgs/Quaternion."""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    q = Quaternion()
    q.x = float(qx)
    q.y = float(qy)
    q.z = float(qz)
    q.w = float(qw)
    return q


def make_pose(x, y, yaw, nav, frame='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, yaw)
    return pose


# Mowing states
STATE_IDLE = "idle"
STATE_BOUNDARY_DISCOVERY = "boundary_discovery"
STATE_NAV_TO_START = "nav_to_start"
STATE_WAITING_FOR_PLAN = "waiting_for_plan"
STATE_MOWING = "mowing"
STATE_REPLANNING = "replanning"
STATE_RETURNING = "returning"
STATE_COMPLETE = "complete"
STATE_STOPPED = "stopped"


class GeminiMowExecutor(Node):
    """Demo-ready mowing executor with visualization and all features."""

    def __init__(self, navigator: BasicNavigator):
        super().__init__('gemini_mow_executor')
        self.nav = navigator

        # ---------- Parameters ----------
        self.declare_parameter('auto_start', False)
        self.declare_parameter('batch_size', 20)
        self.declare_parameter('max_replan_attempts', 3)
        self.declare_parameter('mow_mark_radius', 0.5)
        self.declare_parameter('plan_timeout', 60.0)
        self.declare_parameter('stuck_timeout', 15.0)
        self.declare_parameter('stuck_distance', 0.08)

        self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.batch_size = self.get_parameter('batch_size').get_parameter_value().integer_value
        self.max_replan_attempts = self.get_parameter('max_replan_attempts').get_parameter_value().integer_value
        self.mow_mark_radius = self.get_parameter('mow_mark_radius').get_parameter_value().double_value
        self.plan_timeout = self.get_parameter('plan_timeout').get_parameter_value().double_value
        self.stuck_timeout = self.get_parameter('stuck_timeout').get_parameter_value().double_value
        self.stuck_distance = self.get_parameter('stuck_distance').get_parameter_value().double_value

        # ---------- State ----------
        self.state = STATE_IDLE
        self.coverage_waypoints = []
        self.perimeter_waypoints = []
        self.sweep_waypoints = []
        self.hazard_data = None
        self.coverage_status = None
        self.missed_waypoints = []
        self.replan_count = 0
        self.total_waypoints = 0
        self.total_completed = 0
        self.start_time = 0.0
        self.dock_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.frontiers_explored = 0
        self.current_pass = 1
        self.last_position = None
        self.last_move_time = 0.0
        self.recovery_count = 0

        self.cb_group = ReentrantCallbackGroup()

        # ---------- Subscribers ----------
        self.create_subscription(String, '/mow_command', self._command_cb, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/coverage/waypoints', self._coverage_wp_cb, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/coverage/status', self._coverage_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/gemini/hazards', self._hazard_cb, 10, callback_group=self.cb_group)

        # ---------- Publishers ----------
        self.replan_pub = self.create_publisher(String, '/coverage/replan', 10)
        self.progress_pub = self.create_publisher(String, '/mow_progress', 10)
        self.mark_mowed_pub = self.create_publisher(String, '/coverage/mark_mowed', 10)
        self.blade_pub = self.create_publisher(Bool, '/mower_blade_state', 10)
        self.painter_pub = self.create_publisher(String, '/mower_painter/image', 10)

        # Visualization publishers
        self.trail_pub = self.create_publisher(Marker, '/viz/robot_trail', 10)
        self.planned_path_pub = self.create_publisher(Marker, '/viz/planned_path', 10)
        self.frontier_pub = self.create_publisher(MarkerArray, '/viz/frontiers', 10)
        self.deviation_pub = self.create_publisher(Marker, '/viz/deviation', 10)
        self.status_marker_pub = self.create_publisher(Marker, '/viz/status_text', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/viz/waypoints', 10)

        # ---------- TF & Path Tracking ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Trail data
        self.trail_points = []  # [(x, y, state)]
        self.deviation_points = []  # [(x, y)] when deviating

        # Timers
        self.create_timer(0.5, self._update_trail, callback_group=self.cb_group)
        self.create_timer(1.0, self._publish_status_overlay, callback_group=self.cb_group)

        self.get_logger().info(
            "GeminiMowExecutor ready — send a command to /mow_command to begin"
        )
        
        # Fire a one-shot timer to immediately clear any ghost markers left by RViz
        # Wait until 5 seconds have passed so RViz has time to subscribe to the topics
        self.cleanup_timer = self.create_timer(5.0, self._initial_cleanup_cb, callback_group=self.cb_group)
        self.cleanup_attempts = 0

    def _initial_cleanup_cb(self):
        """Timer callback to run visual cleanup at boot, repeating a few times to ensure reception."""
        self.get_logger().info("🧹 Performing initial RViz marker cleanup...")
        self._clear_rviz_markers()
        self.cleanup_attempts += 1
        
        # Try 3 times, spaced 5 seconds apart, to guarantee RViz gets the memo
        if self.cleanup_attempts >= 3:
            self.cleanup_timer.cancel()

    # ================================================================== #
    # Robot Position Helper
    # ================================================================== #
    def _get_robot_position(self):
        """Get current robot (x, y, yaw) from TF."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (x, y, yaw)
        except Exception:
            return None

    # ================================================================== #
    # Stuck Detection & Recovery
    # ================================================================== #
    def _check_stuck_and_recover(self):
        """Check if robot is stuck and trigger recovery if needed."""
        pos = self._get_robot_position()
        if pos is None:
            return False

        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_position is not None:
            dx = pos[0] - self.last_position[0]
            dy = pos[1] - self.last_position[1]
            dist = math.hypot(dx, dy)

            if dist > self.stuck_distance:
                self.last_position = pos
                self.last_move_time = now
                return False

            if (now - self.last_move_time) > self.stuck_timeout:
                self.recovery_count += 1

                # Give up after 5 recovery attempts — skip the current goal
                if self.recovery_count > 5:
                    self.get_logger().error(
                        "🛑 Max recoveries reached (5). Skipping current goal."
                    )
                    self.nav.cancelTask()
                    time.sleep(0.5)
                    self.last_position = self._get_robot_position()
                    self.last_move_time = self.get_clock().now().nanoseconds / 1e9
                    self.recovery_count = 0
                    return True

                self.get_logger().warn(
                    f"🚨 STUCK DETECTED! No movement for {self.stuck_timeout}s. "
                    f"Triggering recovery (attempt #{self.recovery_count})"
                )

                self._set_blade_state(False)

                # Cancel current task
                self.nav.cancelTask()
                time.sleep(0.5)

                # Try backup
                self.get_logger().info("Recovery: Backing up 0.3m...")
                self.nav.backup(backup_dist=0.3, backup_speed=0.1, time_allowance=10)
                while not self.nav.isTaskComplete():
                    time.sleep(0.1)

                # Try spin
                self.get_logger().info("Recovery: Spinning 90°...")
                self.nav.spin(spin_dist=1.57, time_allowance=10)
                while not self.nav.isTaskComplete():
                    time.sleep(0.1)

                self.last_position = self._get_robot_position()
                self.last_move_time = self.get_clock().now().nanoseconds / 1e9
                return True
        else:
            self.last_position = pos
            self.last_move_time = now

        return False

    # ================================================================== #
    # Callbacks
    # ================================================================== #
    def _command_cb(self, msg):
        """Handle mow commands — starts the full pipeline or adapts mid-run."""
        command = msg.data.strip().lower()
        if not command:
            return

        self.get_logger().info(f"Command received: '{command}'")

        # Mid-run adaptive commands
        if self.state == STATE_MOWING:
            if any(w in command for w in ['stop', 'halt', 'pause']):
                self.get_logger().info("STOP command — halting mow")
                self.nav.cancelTask()
                self.state = STATE_STOPPED
                self._publish_progress("stopped_by_user")
                return
            return

        # New mow command — start the full pipeline
        if self.state not in (STATE_IDLE, STATE_COMPLETE, STATE_STOPPED):
            self.get_logger().warn(f"Already in state '{self.state}' — ignoring")
            return

        self.state = STATE_IDLE
        self.replan_count = 0
        self.missed_waypoints = []
        self.total_completed = 0
        self.frontiers_explored = 0
        self.current_pass = 1
        self.recovery_count = 0
        self.trail_points = []
        self.deviation_points = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Save dock position
        pos = self._get_robot_position()
        if pos:
            self.dock_position = {'x': pos[0], 'y': pos[1], 'yaw': pos[2]}

        self._clear_rviz_markers()
        self._run_boundary_discovery()

    def _clear_rviz_markers(self):
        """Send DELETEALL action to all visual markers to clear ghost trails from previous runs."""
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        
        # Clear trail
        clear.ns = 'robot_trail'
        self.trail_pub.publish(clear)
        
        # Clear planned waypoints
        clear_array = MarkerArray()
        clear_wp = Marker()
        clear_wp.header = clear.header
        clear_wp.ns = 'planned_waypoints'
        clear_wp.action = Marker.DELETEALL
        clear_array.markers.append(clear_wp)
        self.waypoint_pub.publish(clear_array)
        
        # Clear frontiers
        clear_frontier = MarkerArray()
        clear_f = Marker()
        clear_f.header = clear.header
        clear_f.ns = 'frontier_target'
        clear_f.action = Marker.DELETEALL
        clear_frontier.markers.append(clear_f)
        self.frontier_pub.publish(clear_frontier)

    def _set_blade_state(self, is_active: bool):
        """Turn the virtual mower blade on or off."""
        msg = Bool()
        msg.data = is_active
        self.blade_pub.publish(msg)
        
        if is_active:
            self.get_logger().info("⚔️ Blades: ON")
        else:
            self.get_logger().info("⚔️ Blades: OFF")

    def _coverage_wp_cb(self, msg):
        """Receive coverage waypoints or frontier targets from the planner."""
        try:
            data = json.loads(msg.data)
            source = data.get('source', '')

            # --- PHASE 1: FRONTIER EXPLORATION LOOP ---
            if source == 'frontier_explorer':
                if data.get('complete', False):
                    self.get_logger().info(
                        f"✅ Boundary discovery COMPLETE after {self.frontiers_explored} frontiers."
                    )
                    time.sleep(3.0)  # Let SLAM finalize

                    print("=" * 60)
                    print(f"  PHASE 2: COVERAGE MOWING (Pass {self.current_pass})")
                    print("  Requesting coverage plan from BCD planner...")
                    print("=" * 60)

                    self.state = STATE_WAITING_FOR_PLAN
                    self._publish_progress("waiting_for_plan")

                    replan_msg = String()
                    replan_msg.data = json.dumps({"trigger": "boundary_discovery_complete"})
                    self.replan_pub.publish(replan_msg)
                    return

                waypoints = data.get('waypoints', [])
                if not waypoints:
                    self.get_logger().info("No frontiers available yet, retrying in 3s...")
                    time.sleep(3.0)
                    replan_msg = String()
                    replan_msg.data = json.dumps({"command": "get_frontier"})
                    self.replan_pub.publish(replan_msg)
                    return

                fp = waypoints[0]
                self.frontiers_explored += 1

                # Safety: cap frontier exploration to prevent infinite loops
                if self.frontiers_explored > 30:
                    self.get_logger().warn(
                        "⚠️ Explored 30 frontiers — force-completing boundary discovery."
                    )
                    self.state = STATE_WAITING_FOR_PLAN
                    self._publish_progress("waiting_for_plan")
                    replan_msg = String()
                    replan_msg.data = json.dumps({"trigger": "boundary_discovery_complete"})
                    self.replan_pub.publish(replan_msg)
                    return

                self.get_logger().info(
                    f"🔍 Frontier #{self.frontiers_explored}: ({fp['x']:.2f}, {fp['y']:.2f})"
                )

                # Visualize frontier target
                self._publish_frontier_marker(fp['x'], fp['y'])

                wp_pose = make_pose(fp['x'], fp['y'], fp['yaw'], self.nav)
                self.nav.goToPose(wp_pose)

                # Wait with stuck detection
                stuck_check_count = 0
                while not self.nav.isTaskComplete():
                    time.sleep(0.2)
                    stuck_check_count += 1
                    if stuck_check_count % 50 == 0:  # Check every 10s
                        if self._check_stuck_and_recover():
                            break  # Recovery triggered, request new frontier

                result = self.nav.getResult()
                replan_data = {"command": "get_frontier"}
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("✅ Reached Frontier successfully.")
                else:
                    self.get_logger().info("⚠️ Frontier imprecise, continuing anyway.")
                
                # ALWAYS blacklist the frontier we just explored.
                # If SLAM cleared it, it won't be found again anyway.
                # If SLAM didn't clear it (e.g. it's behind a wall), this prevents an infinite loop!
                replan_data["failed_frontier"] = fp

                time.sleep(1.0)
                replan_msg = String()
                replan_msg.data = json.dumps(replan_data)
                self.replan_pub.publish(replan_msg)
                return

            # --- PHASE 2/3: COVERAGE MOWING WAYPOINTS ---
            # GUARD: Do NOT process coverage waypoints if we're still discovering boundaries.
            # The GeminiMowPlanner also subscribes to /mow_command and immediately fires
            # a fallback plan which triggers coverage generation before discovery finishes.
            if self.state == STATE_BOUNDARY_DISCOVERY:
                self.get_logger().info(
                    "⏳ Ignoring coverage waypoints — still in boundary discovery phase"
                )
                return

            if data.get('complete', False):
                self.get_logger().info("✅ Coverage planner says: ALL COVERED!")
                self._run_return_to_dock()
                return

            waypoints = data.get('waypoints', [])
            if not waypoints:
                self.get_logger().warn("Coverage planner returned 0 waypoints")
                self._run_return_to_dock()
                return

            perimeter_count = data.get('perimeter_count', 0)
            self.get_logger().info(f"DEBUG: Parsed perimeter_count = {perimeter_count} from JSON")
            
            # Split into perimeter and sweep waypoints
            self.perimeter_waypoints = waypoints[:perimeter_count] if perimeter_count > 0 else []
            self.sweep_waypoints = waypoints[perimeter_count:]
            self.coverage_waypoints = waypoints
            self.total_waypoints = self.total_completed + len(waypoints)
            
            self.get_logger().info(
                f"📋 Received {len(waypoints)} waypoints "
                f"({len(self.perimeter_waypoints)} perimeter + {len(self.sweep_waypoints)} sweep)"
            )

            # Visualize planned path
            self._publish_planned_path(waypoints)

            if self.state in (STATE_WAITING_FOR_PLAN, STATE_REPLANNING, STATE_MOWING):
                self._run_coverage_mowing()

        except Exception as e:
            self.get_logger().error(f"Failed to parse planner waypoints: {e}")

    def _coverage_status_cb(self, msg):
        try:
            self.coverage_status = json.loads(msg.data)
        except Exception:
            pass

    def _hazard_cb(self, msg):
        try:
            self.hazard_data = json.loads(msg.data)
        except Exception:
            pass

    # ================================================================== #
    # Phase 1: Boundary Discovery (Frontier Exploration)
    # ================================================================== #
    def _run_boundary_discovery(self):
        """Explore frontiers to build the SLAM map."""
        self.state = STATE_BOUNDARY_DISCOVERY
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        print("=" * 60)
        print("  PHASE 1: BOUNDARY DISCOVERY (Frontier Exploration)")
        print("  Actively hunting frontiers to build SLAM map...")
        print("=" * 60)

        self._set_blade_state(False)
        self._publish_progress("boundary_discovery")
        time.sleep(2.0)

        # Do a 360 spin to build a good initial map!
        self.get_logger().info("Spinning 360 to build initial map...")
        self.nav.spin(spin_dist=6.28, time_allowance=20)
        while not self.nav.isTaskComplete():
            time.sleep(0.5)

        replan_msg = String()
        replan_msg.data = json.dumps({"command": "get_frontier"})
        self.replan_pub.publish(replan_msg)

    # ================================================================== #
    # Phase 2: Coverage Mowing
    # ================================================================== #
    def _run_coverage_mowing(self):
        """Execute coverage waypoints: perimeter first, then boustrophedon sweeps."""
        self.state = STATE_MOWING
        waypoints = self.coverage_waypoints

        if not waypoints:
            self.get_logger().warn("No waypoints to execute")
            self._run_return_to_dock()
            return

        self.last_position = self._get_robot_position()
        self.last_move_time = self.get_clock().now().nanoseconds / 1e9

        # ── Phase 2a: PERIMETER PASS ──
        perimeter_wps = getattr(self, 'perimeter_waypoints', [])
        if perimeter_wps:
            print("=" * 60)
            print("  PHASE 2a: PERIMETER PASS")
            print(f"  Tracing {len(perimeter_wps)} edge waypoints continuously...")
            print("=" * 60)

            perimeter_poses = [make_pose(wp['x'], wp['y'], wp['yaw'], self.nav) for wp in perimeter_wps]
            self._publish_progress("perimeter")

            # Go to the first point with blades OFF
            self.get_logger().info("Navigating to start of perimeter with blades OFF...")
            self._set_blade_state(False)
            # Reset stuck detection state before starting
            self.recovery_count = 0
            self.last_position = self._get_robot_position()
            self.last_move_time = self.get_clock().now().nanoseconds / 1e9
            self.nav.goToPose(perimeter_poses[0])
            
            nav_stuck_check = 0
            while not self.nav.isTaskComplete():
                time.sleep(0.5)
                nav_stuck_check += 1
                if nav_stuck_check % 20 == 0:  # Check every 10s
                    if self._check_stuck_and_recover():
                        self.get_logger().warn("Recovered while going to start of perimeter! Proceeding to trace anyway.")
                        break
                        
                if self.state == STATE_STOPPED:
                    self.nav.cancelTask()
                    return

            # Turn blades ON and trace the perimeter point-by-point
            self.get_logger().info("Perimeter reached. Turning blades ON and tracing point-by-point...")
            self._set_blade_state(True)
            self.state = STATE_MOWING
            
            for i in range(1, len(perimeter_poses)):
                pose = perimeter_poses[i]
                # Reset stuck detection state for each new waypoint
                self.recovery_count = 0
                self.last_position = self._get_robot_position()
                self.last_move_time = self.get_clock().now().nanoseconds / 1e9
                
                self.nav.goToPose(pose)
                
                nav_stuck_check = 0
                recovered = False
                while not self.nav.isTaskComplete():
                    time.sleep(0.5)
                    nav_stuck_check += 1
                    if nav_stuck_check % 20 == 0:  # Check every 10s
                        if self._check_stuck_and_recover():
                            self.get_logger().warn(f"Recovered at perimeter point {i+1}. Skipping to next point.")
                            recovered = True
                            break
                            
                    if self.state == STATE_STOPPED:
                        self.nav.cancelTask()
                        return
                
                # After recovery, task is already cancelled. Don't call getResult.
                if recovered:
                    continue
                        
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.total_completed += 1
                else:
                    self.get_logger().warn(f"Failed to reach perimeter point {i+1}. Skipping.")
                    
            self.get_logger().info("✅ Perimeter COMPLETE")
                
        # ── Phase 2b: BOUSTROPHEDON SWEEPS ──
        self._set_blade_state(True)
        sweep_wps = self.sweep_waypoints  # Always use only the sweep portion
        if not sweep_wps:
            self.get_logger().info("No sweep waypoints — going to dock")
            self._run_return_to_dock()
            return

        print("=" * 60)
        print("  PHASE 2b: BOUSTROPHEDON SWEEPS")
        print(f"  Mowing {len(sweep_wps)} sweep waypoints...")
        print("=" * 60)

        poses = [make_pose(wp['x'], wp['y'], wp['yaw'], self.nav) for wp in sweep_wps]
        self.total_waypoints = self.total_completed + len(poses)

        self.get_logger().info(
            f"🚜 Sweeping {len(poses)} waypoints in batches of {self.batch_size}"
        )
        self._publish_progress("mowing")

        batch_failed_wps = []
        self.last_position = self._get_robot_position()
        self.last_move_time = self.get_clock().now().nanoseconds / 1e9

        for batch_start in range(0, len(poses), self.batch_size):
            if self.state == STATE_STOPPED:
                return

            self.recovery_count = 0  # Reset per-batch recovery counter
            batch_end = min(batch_start + self.batch_size, len(poses))
            batch = poses[batch_start:batch_end]
            batch_wp_dicts = sweep_wps[batch_start:batch_end]
            batch_num = (batch_start // self.batch_size) + 1
            total_batches = (len(poses) + self.batch_size - 1) // self.batch_size

            self.get_logger().info(
                f"📦 Sweep Batch {batch_num}/{total_batches} ({len(batch)} waypoints)"
            )
            self._publish_progress("mowing")

            # Execute batch using followWaypoints for reliable straight lines
            self.nav.followWaypoints(batch)

            i = 0
            last_wp_reached = 0
            while not self.nav.isTaskComplete():
                i += 1
                time.sleep(0.2)
                feedback = self.nav.getFeedback()
                if feedback and hasattr(feedback, 'current_waypoint'):
                    if feedback.current_waypoint > last_wp_reached:
                        last_wp_reached = feedback.current_waypoint

                if i % 15 == 0:
                    self._publish_progress("mowing")
                    # Mark waypoints as mowed
                    for j in range(last_wp_reached):
                        idx = batch_start + j
                        if idx < len(sweep_wps):
                            self._mark_as_mowed(sweep_wps[idx]['x'], sweep_wps[idx]['y'])

                # Stuck detection every 50 iterations (10s)
                if i % 50 == 0:
                    if self._check_stuck_and_recover():
                        # Skip rest of batch after recovery
                        break

                if self.state == STATE_STOPPED:
                    self.nav.cancelTask()
                    return

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.total_completed += len(batch)
                self.get_logger().info(
                    f"✅ Batch {batch_num} DONE ({self.total_completed}/{self.total_waypoints})"
                )
                for wp in batch_wp_dicts:
                    self._mark_as_mowed(wp['x'], wp['y'])

            elif result == TaskResult.CANCELED:
                self.get_logger().info(f"Batch {batch_num} canceled")
                self.state = STATE_STOPPED
                return

            else:
                self.get_logger().warn(f"⚠️ Batch {batch_num} followWaypoints FAILED or partially failed")
                # Mark as failed what we didn't reach
                for j in range(last_wp_reached, len(batch_wp_dicts)):
                    idx = batch_start + j
                    if idx < len(sweep_wps):
                        batch_failed_wps.append(sweep_wps[idx])
                for j in range(last_wp_reached):
                    idx = batch_start + j
                    if idx < len(sweep_wps):
                        self._mark_as_mowed(sweep_wps[idx]['x'], sweep_wps[idx]['y'])
                self.total_completed += last_wp_reached
                time.sleep(1.0)

        # After all batches: replan to check for missed areas
        self.replan_count += 1
        if self.replan_count <= self.max_replan_attempts:
            self.get_logger().info(
                f"🔄 Replan #{self.replan_count}: checking for missed areas..."
            )
            self.state = STATE_REPLANNING
            self._publish_progress("replanning")
            replan_msg = String()
            replan_msg.data = json.dumps({"missed_waypoints": batch_failed_wps} if batch_failed_wps else {})
            self.replan_pub.publish(replan_msg)
        else:
            self.get_logger().info("Max replans reached, finishing up.")
            self._run_return_to_dock()

    # ================================================================== #
    # Phase 3: Return to Dock
    # ================================================================== #
    def _run_return_to_dock(self):
        """Navigate back to the starting position."""
        self._set_blade_state(False)
        self.state = STATE_RETURNING

        print("=" * 60)
        print("  PHASE 3: RETURNING TO DOCK")
        print(f"  Navigating back to ({self.dock_position['x']:.1f}, {self.dock_position['y']:.1f})")
        print("=" * 60)

        self._publish_progress("returning_to_dock")

        dock_pose = make_pose(
            self.dock_position['x'],
            self.dock_position['y'],
            self.dock_position['yaw'],
            self.nav
        )

        self.nav.goToPose(dock_pose)
        while not self.nav.isTaskComplete():
            time.sleep(0.5)

        result = self.nav.getResult()
        elapsed = (self.get_clock().now().nanoseconds / 1e9 - self.start_time) / 60.0

        self.state = STATE_COMPLETE

        coverage_pct = 0
        if self.coverage_status:
            coverage_pct = self.coverage_status.get('coverage_percent', 0)

        print("=" * 60)
        print("  🎉 MOWING COMPLETE!")
        print(f"  Coverage: {coverage_pct:.1f}%")
        print(f"  Time: {elapsed:.1f} minutes")
        print(f"  Frontiers explored: {self.frontiers_explored}")
        print(f"  Waypoints completed: {self.total_completed}")
        print(f"  Replans: {self.replan_count}")
        print(f"  Recoveries: {self.recovery_count}")
        print("=" * 60)

        self._publish_progress("complete")

    # ================================================================== #
    # Helpers
    # ================================================================== #
    def _mark_as_mowed(self, x, y):
        """Tell coverage planner this area is mowed."""
        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "radius": self.mow_mark_radius})
        self.mark_mowed_pub.publish(msg)

    def _publish_progress(self, status):
        """Publish progress to /mow_progress."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time if self.start_time > 0 else 0
        total = self.total_waypoints if self.total_waypoints > 0 else 1
        pct = min(100.0, self.total_completed / total * 100)

        if self.total_completed > 0 and pct < 100:
            rate = elapsed / self.total_completed
            remaining = total - self.total_completed
            eta = (rate * remaining) / 60.0
        else:
            eta = 0.0

        coverage_pct = 0
        if self.coverage_status:
            coverage_pct = self.coverage_status.get('coverage_percent', 0)

        progress = {
            "state": self.state,
            "status": status,
            "percent_complete": round(pct, 1),
            "waypoints_done": self.total_completed,
            "waypoints_total": total,
            "estimated_minutes_remaining": round(eta, 1),
            "elapsed_seconds": round(elapsed, 1),
            "replan_count": self.replan_count,
            "coverage_percent": round(coverage_pct, 1),
            "frontiers_explored": self.frontiers_explored,
            "recoveries": self.recovery_count,
            "pass": self.current_pass,
        }

        msg = String()
        msg.data = json.dumps(progress)
        self.progress_pub.publish(msg)

        state_emoji = {
            "boundary_discovery": "🔍",
            "waiting_for_plan": "⏳",
            "mowing": "🚜",
            "replanning": "🔄",
            "returning_to_dock": "🏠",
            "complete": "🎉",
            "stopped_by_user": "⏹️",
        }.get(status, "📊")

        self.get_logger().info(
            f"{state_emoji} [{status}] {pct:.0f}% | {self.total_completed}/{total} | "
            f"Coverage: {coverage_pct:.0f}% | ETA: {eta:.1f}min"
        )

    # ================================================================== #
    # RViz Visualization
    # ================================================================== #
    def _update_trail(self):
        """Timer callback: record robot trail and publish visualization."""
        pos = self._get_robot_position()
        if pos is None:
            return

        self.trail_points.append((pos[0], pos[1], self.state))
        # Cap trail to prevent unbounded memory growth on long runs
        if len(self.trail_points) > 5000:
            self.trail_points = self.trail_points[-5000:]
        self._publish_trail_marker()
        
        # Real-time footprint logging: constantly mark ground as mowed during action
        if self.state == STATE_MOWING:
            self._mark_as_mowed(pos[0], pos[1])

    def _publish_trail_marker(self):
        """Publish the robot trail as a colored line strip in RViz."""
        if len(self.trail_points) < 2:
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_trail'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # Line width
        marker.pose.orientation.w = 1.0

        for x, y, state in self.trail_points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.05
            marker.points.append(p)

            c = ColorRGBA()
            if state == STATE_MOWING:
                c.r, c.g, c.b, c.a = 0.0, 0.9, 0.0, 0.9  # Green: mowing
            elif state == STATE_BOUNDARY_DISCOVERY:
                c.r, c.g, c.b, c.a = 0.2, 0.5, 1.0, 0.9  # Blue: exploring
            elif state == STATE_RETURNING:
                c.r, c.g, c.b, c.a = 1.0, 0.8, 0.0, 0.9  # Yellow: returning
            else:
                c.r, c.g, c.b, c.a = 0.5, 0.5, 0.5, 0.5  # Grey: idle
            marker.colors.append(c)

        self.trail_pub.publish(marker)

    def _publish_planned_path(self, waypoints):
        """Publish planned coverage waypoints as yellow spheres."""
        marker_array = MarkerArray()

        # Clear old markers
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = 'planned_waypoints'
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, wp in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'planned_waypoints'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.lifetime.sec = 0  # Never expire
            m.lifetime.nanosec = 0
            m.pose.position.x = float(wp['x'])
            m.pose.position.y = float(wp['y'])
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.9
            m.color.b = 0.0
            m.color.a = 0.6
            marker_array.markers.append(m)

        # Also publish a LINE_STRIP connecting all waypoints for easy route visualization
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'planned_path_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.lifetime.sec = 0  # Never expire
        line.lifetime.nanosec = 0
        line.scale.x = 0.03  # Line width
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.5
        line.color.a = 0.8
        line.pose.orientation.w = 1.0
        for wp in waypoints:
            p = Point()
            p.x = float(wp['x'])
            p.y = float(wp['y'])
            p.z = 0.05
            line.points.append(p)
        marker_array.markers.append(line)

        self.waypoint_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(waypoints)} waypoint markers to RViz")

    def _publish_frontier_marker(self, x, y):
        """Publish frontier target as an orange sphere."""
        marker_array = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontier_target'
        m.id = self.frontiers_explored
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.4
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 0.8
        m.lifetime.sec = 0  # Persist forever
        marker_array.markers.append(m)
        self.frontier_pub.publish(marker_array)

    def _publish_status_overlay(self):
        """Publish a text marker showing current state in RViz."""
        if self.state == STATE_IDLE:
            return

        pos = self._get_robot_position()
        if pos is None:
            return

        coverage_pct = 0
        if self.coverage_status:
            coverage_pct = self.coverage_status.get('coverage_percent', 0)

        total = self.total_waypoints if self.total_waypoints > 0 else 1
        wp_pct = min(100.0, self.total_completed / total * 100)

        state_labels = {
            STATE_BOUNDARY_DISCOVERY: "🔍 EXPLORING",
            STATE_NAV_TO_START: "🚀 NAV TO START",
            STATE_WAITING_FOR_PLAN: "⏳ PLANNING",
            STATE_MOWING: "🚜 MOWING",
            STATE_REPLANNING: "🔄 REPLANNING",
            STATE_RETURNING: "🏠 RETURNING",
            STATE_COMPLETE: "🎉 DONE",
            STATE_STOPPED: "⏹ STOPPED",
        }

        elapsed = (self.get_clock().now().nanoseconds / 1e9 - self.start_time) / 60.0

        text = (
            f"{state_labels.get(self.state, self.state)}\n"
            f"Waypoints: {self.total_completed}/{total} ({wp_pct:.0f}%)\n"
            f"Coverage: {coverage_pct:.0f}%\n"
            f"Time: {elapsed:.1f}min"
        )

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'status_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 1.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.text = text
        self.status_marker_pub.publish(marker)


def main():
    rclpy.init()

    nav = BasicNavigator()
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])

    print("=" * 60)
    print("  🤖 GEMINI AI MOW EXECUTOR — Demo Mode")
    print("  Waiting for Nav2 navigation servers...")
    print("=" * 60)

    nav.waitUntilNav2Active(localizer='robot_localization')
    print("Nav2 is active!")

    print("Waiting 8s for SLAM to initialize with LiDAR scans...")
    time.sleep(8.0)

    executor_node = GeminiMowExecutor(nav)
    executor_node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    print("\n🏁 Auto-starting mowing sequence...")
    executor_node._command_cb(String(data='auto_mow'))

    try:
        executor = MultiThreadedExecutor()
        # NOTE: Do NOT add nav (BasicNavigator) — it uses
        # rclpy.spin_until_future_complete() internally.
        executor.add_node(executor_node)
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
