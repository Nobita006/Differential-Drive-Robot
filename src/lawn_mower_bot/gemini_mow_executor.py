#!/usr/bin/env python3
"""
Gemini AI Mow Executor — Real-World Ready

Executes mowing in three phases:
  Phase 1: Boundary Discovery — drives a perimeter to let SLAM build the map
  Phase 2: Coverage Mowing — receives waypoints from coverage_planner, executes
  Phase 3: Cleanup — re-plans for missed areas, fills gaps

Integrates with:
  - coverage_planner (BCD waypoints from SLAM costmap)
  - gemini_planner (real-time hazard detection)
  - gemini_mow_planner (NLP command parsing)

Subscribes:
    /mow_command         (String)  — natural language mowing commands
    /coverage/waypoints  (String)  — coverage waypoints from planner
    /coverage/status     (String)  — coverage statistics
    /gemini/hazards      (String)  — real-time hazard alerts
    /gemini/mow_plan     (String)  — AI-parsed plan (forwarded to coverage planner)

Publishes:
    /mow_progress        (String)  — JSON progress updates
    /coverage/mark_mowed (String)  — marks areas as mowed
    /coverage/replan     (String)  — requests re-planning
"""
import rclpy
import math
import time
import json
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from rclpy.parameter import Parameter


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


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
STATE_WAITING_FOR_PLAN = "waiting_for_plan"
STATE_MOWING = "mowing"
STATE_REPLANNING = "replanning"
STATE_COMPLETE = "complete"
STATE_STOPPED = "stopped"


class GeminiMowExecutor(Node):
    """Real-world mowing executor with boundary discovery and reactive re-planning."""

    def __init__(self, navigator: BasicNavigator):
        super().__init__('gemini_mow_executor')
        self.nav = navigator

        # ---------- Parameters ----------
        self.declare_parameter('auto_start', True)
        self.declare_parameter('boundary_radius', 5.0)
        self.declare_parameter('boundary_steps', 20)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('max_replan_attempts', 3)
        self.declare_parameter('mow_mark_radius', 0.5)
        self.declare_parameter('plan_timeout', 30.0)

        self.auto_start = self.get_parameter(
            'auto_start').get_parameter_value().bool_value

        self.boundary_radius = self.get_parameter(
            'boundary_radius').get_parameter_value().double_value
        self.boundary_steps = self.get_parameter(
            'boundary_steps').get_parameter_value().integer_value
        self.batch_size = self.get_parameter(
            'batch_size').get_parameter_value().integer_value
        self.max_replan_attempts = self.get_parameter(
            'max_replan_attempts').get_parameter_value().integer_value
        self.mow_mark_radius = self.get_parameter(
            'mow_mark_radius').get_parameter_value().double_value
        self.plan_timeout = self.get_parameter(
            'plan_timeout').get_parameter_value().double_value

        # ---------- State ----------
        self.state = STATE_IDLE
        self.coverage_waypoints = []
        self.hazard_data = None
        self.coverage_status = None
        self.missed_waypoints = []
        self.replan_count = 0
        self.start_time = 0.0
        self.total_completed = 0
        self.total_waypoints = 0

        # ---------- Subscribers ----------
        self.create_subscription(String, '/mow_command', self._command_cb, 10)
        self.create_subscription(String, '/coverage/waypoints', self._coverage_wp_cb, 10)
        self.create_subscription(String, '/coverage/status', self._coverage_status_cb, 10)
        self.create_subscription(String, '/gemini/hazards', self._hazard_cb, 10)

        # ---------- Publishers ----------
        self.progress_pub = self.create_publisher(String, '/mow_progress', 10)
        self.mark_mowed_pub = self.create_publisher(String, '/coverage/mark_mowed', 10)
        self.replan_pub = self.create_publisher(String, '/coverage/replan', 10)

        self.get_logger().info(
            "GeminiMowExecutor ready — send a command to /mow_command to begin"
        )

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
            elif any(w in command for w in ['quick', 'fast', 'hurry', 'rain']):
                self.get_logger().info("HURRY command — will skip alternate waypoints")
                self.coverage_waypoints = self.coverage_waypoints[::2]
                self.total_waypoints = self.total_completed + len(self.coverage_waypoints)
                return
            # Other mid-run commands: the mow_planner will parse and publish a new plan
            return

        # New mow command — start the full pipeline
        if self.state not in (STATE_IDLE, STATE_COMPLETE, STATE_STOPPED):
            self.get_logger().warn(
                f"Already in state '{self.state}' — ignoring new command"
            )
            return

        self.state = STATE_IDLE
        self.replan_count = 0
        self.missed_waypoints = []
        self.total_completed = 0
        self.start_time = time.time()

        # Start Phase 1: Boundary Discovery
        self._run_boundary_discovery()

    def _coverage_wp_cb(self, msg):
        """Receive coverage waypoints from the planner."""
        try:
            data = json.loads(msg.data)

            if data.get('complete', False):
                self.get_logger().info("Coverage planner says: ALL COVERED!")
                self.state = STATE_COMPLETE
                self._publish_progress("complete")
                return

            waypoints = data.get('waypoints', [])
            if not waypoints:
                self.get_logger().warn("Coverage planner returned 0 waypoints")
                return

            self.coverage_waypoints = waypoints
            self.total_waypoints = self.total_completed + len(waypoints)
            self.get_logger().info(
                f"Received {len(waypoints)} coverage waypoints from planner"
            )

            # If we were waiting for a plan, start mowing
            if self.state in (STATE_WAITING_FOR_PLAN, STATE_REPLANNING):
                self._run_coverage_mowing()

        except Exception as e:
            self.get_logger().error(f"Failed to parse coverage waypoints: {e}")

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
    # Phase 1: Boundary Discovery
    # ================================================================== #
    def _run_boundary_discovery(self):
        """
        Drive a perimeter loop so SLAM can map the lawn boundaries.
        Uses L-shaped waypoints that follow the world perimeter.
        """
        self.state = STATE_BOUNDARY_DISCOVERY
        self.start_time = time.time()

        print("=" * 60)
        print("  PHASE 1: BOUNDARY DISCOVERY")
        print("  Driving L-shaped perimeter for SLAM mapping...")
        print("=" * 60)

        self._publish_progress("boundary_discovery")

        r = self.boundary_radius
        # L-shaped perimeter: main rect + extension arm
        # Main rectangle: (-r, -r) to (r, r*0.5)
        # Extension: (-r, r*0.5) to (r*0.2, r)
        perimeter_points = [
            # Start near origin, go east along south side
            (0.0, 0.0, 0.0),
            (r * 0.8, 0.0, 0.0),
            (r * 0.8, -r * 0.8, -math.pi/2),
            # South-east corner, go west
            (r * 0.8, -r * 0.8, math.pi),
            (-r * 0.8, -r * 0.8, math.pi),
            # South-west corner, go north
            (-r * 0.8, -r * 0.8, math.pi/2),
            (-r * 0.8, 0.0, math.pi/2),
            (-r * 0.8, r * 0.8, math.pi/2),
            # Extension top, go east
            (-r * 0.8, r * 0.8, 0.0),
            (0.0, r * 0.8, 0.0),
            # Inner corner, go south then east
            (0.0, r * 0.4, -math.pi/2),
            (r * 0.8, r * 0.4, 0.0),
            # North-east of main area, go south to close
            (r * 0.8, 0.0, -math.pi/2),
            # Return near start
            (0.5, 0.0, math.pi),
        ]

        perimeter_wps = [
            make_pose(x, y, yaw, self.nav)
            for x, y, yaw in perimeter_points
        ]

        self.get_logger().info(
            f"Starting boundary discovery with {len(perimeter_wps)} waypoints"
        )

        self.nav.followWaypoints(perimeter_wps)

        i = 0
        while not self.nav.isTaskComplete():
            i += 1
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback = self.nav.getFeedback()
            if feedback and i % 30 == 0:
                current_wp = feedback.current_waypoint
                total = len(perimeter_wps)
                self.get_logger().info(
                    f"Boundary discovery: {current_wp}/{total} waypoints"
                )
                self._publish_progress("boundary_discovery")
                if current_wp < total:
                    wp = perimeter_wps[current_wp]
                    self._mark_as_mowed(
                        wp.pose.position.x, wp.pose.position.y
                    )

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Boundary discovery COMPLETE")
        elif result == TaskResult.FAILED:
            self.get_logger().warn(
                "Boundary discovery partially failed — proceeding with available map"
            )
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Boundary discovery canceled")
            self.state = STATE_STOPPED
            return

        self.get_logger().info("Waiting 3s for SLAM to finalize map...")
        time.sleep(3.0)

        # Transition to Phase 2
        print("=" * 60)
        print("  PHASE 2: COVERAGE MOWING")
        print("  Requesting coverage plan from BCD planner...")
        print("=" * 60)

        self.state = STATE_WAITING_FOR_PLAN
        self._publish_progress("waiting_for_plan")

        # Trigger the coverage planner
        replan_msg = String()
        replan_msg.data = json.dumps({"trigger": "boundary_discovery_complete"})
        self.replan_pub.publish(replan_msg)

        # Fallback: if coverage planner doesn't respond, generate waypoints
        self._wait_for_plan_or_fallback()

    # ================================================================== #
    # Fallback Plan Generation
    # ================================================================== #
    def _wait_for_plan_or_fallback(self):
        """Wait for coverage planner response; generate fallback if timeout."""
        deadline = time.time() + self.plan_timeout
        self.get_logger().info(
            f"Waiting up to {self.plan_timeout}s for coverage planner..."
        )

        while self.state == STATE_WAITING_FOR_PLAN:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() > deadline:
                self.get_logger().warn(
                    "Coverage planner timed out — using fallback waypoints"
                )
                fallback = self._generate_fallback_waypoints()
                self.coverage_waypoints = fallback
                self.total_waypoints = len(fallback)
                self._run_coverage_mowing()
                return

        # If we exited the loop, _coverage_wp_cb handled it
        self.get_logger().info("Coverage planner responded in time")

    def _generate_fallback_waypoints(self):
        """
        Generate boustrophedon waypoints for the L-shaped lawn.
        Main area: X[-8,8] Y[-8,3] and Extension: X[-8,0] Y[3,8].
        """
        waypoints = []
        lane_width = 1.0

        # Main rectangular area
        y = -8.0
        direction = 1
        while y <= 3.0:
            if direction == 1:
                waypoints.append({'x': -8.0, 'y': y, 'yaw': 0.0})
                waypoints.append({'x': 8.0, 'y': y, 'yaw': 0.0})
            else:
                waypoints.append({'x': 8.0, 'y': y, 'yaw': math.pi})
                waypoints.append({'x': -8.0, 'y': y, 'yaw': math.pi})
            direction *= -1
            y += lane_width

        # Extension arm (upper-left L section)
        y = 4.0
        while y <= 8.0:
            if direction == 1:
                waypoints.append({'x': -8.0, 'y': y, 'yaw': 0.0})
                waypoints.append({'x': 0.0, 'y': y, 'yaw': 0.0})
            else:
                waypoints.append({'x': 0.0, 'y': y, 'yaw': math.pi})
                waypoints.append({'x': -8.0, 'y': y, 'yaw': math.pi})
            direction *= -1
            y += lane_width

        self.get_logger().info(
            f"Generated {len(waypoints)} fallback waypoints "
            f"(main + extension)"
        )
        return waypoints

    # ================================================================== #
    # Phase 2: Coverage Mowing
    # ================================================================== #
    def _run_coverage_mowing(self):
        """Execute coverage waypoints from the BCD planner in batches."""
        self.state = STATE_MOWING
        waypoints = self.coverage_waypoints

        if not waypoints:
            self.get_logger().warn("No waypoints to execute")
            self.state = STATE_COMPLETE
            return

        # Convert to PoseStamped
        poses = [make_pose(wp['x'], wp['y'], wp['yaw'], self.nav) for wp in waypoints]
        self.total_waypoints = self.total_completed + len(poses)

        self.get_logger().info(
            f"Executing {len(poses)} coverage waypoints in batches of {self.batch_size}"
        )
        self._publish_progress("mowing")

        batch_failed_wps = []

        for batch_start in range(0, len(poses), self.batch_size):
            # Check if we were stopped
            if self.state == STATE_STOPPED:
                return

            batch_end = min(batch_start + self.batch_size, len(poses))
            batch = poses[batch_start:batch_end]
            batch_wp_dicts = waypoints[batch_start:batch_end]
            batch_num = (batch_start // self.batch_size) + 1
            total_batches = (len(poses) + self.batch_size - 1) // self.batch_size

            # ------- Hazard check -------
            if self.hazard_data:
                action = self.hazard_data.get('recommended_action', 'continue')
                n_high = self.hazard_data.get('high_danger_count', 0)
                if action == 'stop' and n_high > 0:
                    self.get_logger().warn(
                        f"[Hazard] HIGH DANGER ({n_high}) — pausing 5s"
                    )
                    self._publish_progress("paused_hazard")
                    time.sleep(5.0)

            self.get_logger().info(
                f"Batch {batch_num}/{total_batches} ({len(batch)} waypoints)"
            )
            self._publish_progress("mowing")

            # ------- Execute batch -------
            self.nav.followWaypoints(batch)

            i = 0
            last_wp_reached = 0
            while not self.nav.isTaskComplete():
                i += 1
                rclpy.spin_once(self, timeout_sec=0.1)
                feedback = self.nav.getFeedback()
                if feedback and i % 30 == 0:
                    current_wp = feedback.current_waypoint
                    last_wp_reached = current_wp
                    overall = self.total_completed + batch_start + current_wp
                    self._publish_progress("mowing")

                    # Mark waypoints as we pass them
                    for j in range(current_wp):
                        idx = batch_start + j
                        if idx < len(waypoints):
                            self._mark_as_mowed(
                                waypoints[idx]['x'], waypoints[idx]['y']
                            )

                    # Check for state changes (stop command)
                    if self.state == STATE_STOPPED:
                        self.nav.cancelTask()
                        return

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.total_completed += len(batch)
                self.get_logger().info(
                    f"Batch {batch_num} DONE ({self.total_completed}/{self.total_waypoints})"
                )
                # Mark all batch waypoints as mowed
                for wp in batch_wp_dicts:
                    self._mark_as_mowed(wp['x'], wp['y'])

            elif result == TaskResult.FAILED:
                self.get_logger().warn(f"Batch {batch_num} FAILED")
                # Track missed waypoints for re-planning
                for j in range(last_wp_reached, len(batch_wp_dicts)):
                    idx = batch_start + j
                    if idx < len(waypoints):
                        batch_failed_wps.append(waypoints[idx])
                # Mark successful ones
                for j in range(last_wp_reached):
                    idx = batch_start + j
                    if idx < len(waypoints):
                        self._mark_as_mowed(waypoints[idx]['x'], waypoints[idx]['y'])
                self.total_completed += last_wp_reached
                time.sleep(2.0)

            elif result == TaskResult.CANCELED:
                self.get_logger().info(f"Batch {batch_num} canceled")
                self.state = STATE_STOPPED
                return

        # ------- Phase 3: Re-plan for missed areas -------
        if batch_failed_wps and self.replan_count < self.max_replan_attempts:
            self.replan_count += 1
            self.get_logger().info(
                f"Re-planning for {len(batch_failed_wps)} missed waypoints "
                f"(attempt {self.replan_count}/{self.max_replan_attempts})"
            )
            self.state = STATE_REPLANNING
            self._publish_progress("replanning")

            replan_msg = String()
            replan_msg.data = json.dumps({
                "missed_waypoints": batch_failed_wps,
                "attempt": self.replan_count,
            })
            self.replan_pub.publish(replan_msg)
            # Coverage planner will respond with new waypoints → _coverage_wp_cb
            return

        # ------- Final report -------
        self.state = STATE_COMPLETE
        elapsed = time.time() - self.start_time

        print("\n" + "=" * 60)
        print("  MOWING COMPLETE!")
        print(f"  Waypoints executed: {self.total_completed}")
        if batch_failed_wps:
            print(f"  Missed waypoints: {len(batch_failed_wps)}")
        print(f"  Total time: {elapsed/60:.1f} minutes")
        print(f"  Re-plan attempts: {self.replan_count}")
        if self.coverage_status:
            print(f"  Coverage: {self.coverage_status.get('coverage_percent', '?')}%")
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
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0
        total = self.total_waypoints if self.total_waypoints > 0 else 1
        pct = min(100.0, self.total_completed / total * 100)

        if self.total_completed > 0 and pct < 100:
            rate = elapsed / self.total_completed
            remaining = total - self.total_completed
            eta = (rate * remaining) / 60.0
        else:
            eta = 0.0

        progress = {
            "state": self.state,
            "status": status,
            "percent_complete": round(pct, 1),
            "waypoints_done": self.total_completed,
            "waypoints_total": total,
            "estimated_minutes_remaining": round(eta, 1),
            "elapsed_seconds": round(elapsed, 1),
            "replan_count": self.replan_count,
        }

        if self.coverage_status:
            progress["coverage_percent"] = self.coverage_status.get(
                'coverage_percent', 0
            )

        msg = String()
        msg.data = json.dumps(progress)
        self.progress_pub.publish(msg)

        self.get_logger().info(
            f"[{status}] {pct:.0f}% | {self.total_completed}/{total} | "
            f"ETA: {eta:.1f}min"
        )


def main():
    rclpy.init()

    nav = BasicNavigator()
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])

    print("=" * 60)
    print("  GEMINI AI MOW EXECUTOR — Real-World Ready")
    print("  Waiting for Nav2 navigation servers...")
    print("=" * 60)

    # With SLAM toolbox, we skip the localizer check entirely.
    # SLAM provides map->odom, local EKF provides odom->base_footprint.
    # 'robot_localization' is the special flag that skips the localizer wait
    # in BasicNavigator (it's a non-lifecycle node, so it skips _waitForNodeToActivate).
    nav.waitUntilNav2Active(localizer='robot_localization')
    print("Nav2 is active!")

    # No setInitialPose needed — SLAM starts at origin automatically.
    # Just give SLAM a moment to initialize with first scans.
    print("Waiting 8s for SLAM to initialize with LiDAR scans...")
    time.sleep(8.0)

    executor_node = GeminiMowExecutor(nav)

    if executor_node.auto_start:
        print("\nAuto-starting mowing sequence...")
        auto_msg = String()
        auto_msg.data = "Mow the yard"
        executor_node._command_cb(auto_msg)
    else:
        print("\nReady! Send a mowing command:")
        print("  ros2 topic pub --once /mow_command std_msgs/msg/String "
              "\"data: 'Mow the yard'\"")

    try:
        rclpy.spin(executor_node)
    except KeyboardInterrupt:
        pass

    executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
