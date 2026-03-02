#!/usr/bin/env python3
"""
Gemini AI Mow Planner — Natural Language → Intelligent Mowing Plan

Accepts natural language commands (e.g. "Mow the backyard, skip the flower beds")
via the /mow_command topic, sends them along with the latest camera frame to the
Gemini API, and publishes a structured mowing plan (waypoints, exclusion zones,
strategy) on /gemini/mow_plan.

Subscribes:
    /mow_command        (String)  — natural language mowing instruction
    /camera/image_raw   (Image)   — latest camera frame for scene context
    /gemini/objects      (String)  — semantic object detections from gemini_planner

Publishes:
    /gemini/mow_plan    (String)  — JSON mowing plan with waypoints & exclusion zones
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import math
import os


class GeminiMowPlanner(Node):
    def __init__(self):
        super().__init__('gemini_mow_planner')

        # ---------- Parameters ----------
        self.declare_parameter('api_key', '')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('yard_x_min', -9.5)
        self.declare_parameter('yard_x_max', 9.5)
        self.declare_parameter('yard_y_min', -9.5)
        self.declare_parameter('yard_y_max', 9.5)
        self.declare_parameter('safe_margin', 2.5)
        self.declare_parameter('default_lane_width', 0.3)

        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.yard_x_min = self.get_parameter('yard_x_min').get_parameter_value().double_value
        self.yard_x_max = self.get_parameter('yard_x_max').get_parameter_value().double_value
        self.yard_y_min = self.get_parameter('yard_y_min').get_parameter_value().double_value
        self.yard_y_max = self.get_parameter('yard_y_max').get_parameter_value().double_value
        self.safe_margin = self.get_parameter('safe_margin').get_parameter_value().double_value
        self.default_lane_width = self.get_parameter('default_lane_width').get_parameter_value().double_value

        if not self.api_key:
            self.api_key = os.environ.get('GEMINI_API_KEY', '')

        # ---------- Gemini API ----------
        self.api_available = False
        if self.api_key:
            try:
                from google import genai
                self.client = genai.Client(api_key=self.api_key)
                self.model_name = "gemini-robotics-er-1.5-preview"
                self.api_available = True
                self.get_logger().info(f"Gemini API ready (model: {self.model_name})")
            except Exception as e:
                self.get_logger().error(f"Gemini API init failed: {e}")
        else:
            self.get_logger().warn("No API key — will use fallback planner only")

        # ---------- Subscribers ----------
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_objects = None

        self.create_subscription(Image, camera_topic, self._image_cb, 10)
        self.create_subscription(String, '/mow_command', self._command_cb, 10)
        self.create_subscription(String, '/gemini/objects', self._objects_cb, 10)

        # ---------- Publisher ----------
        self.plan_pub = self.create_publisher(String, '/gemini/mow_plan', 10)

        self.get_logger().info("GeminiMowPlanner ready — publish to /mow_command to start")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def _objects_cb(self, msg):
        try:
            self.latest_objects = json.loads(msg.data)
        except Exception:
            pass

    def _command_cb(self, msg):
        command = msg.data.strip()
        if not command:
            return
        self.get_logger().info(f"Received mow command: '{command}'")
        plan = self._generate_plan(command)
        if plan:
            out = String()
            out.data = json.dumps(plan)
            self.plan_pub.publish(out)
            self.get_logger().info(
                f"Published plan: {plan.get('plan_summary', 'N/A')} "
                f"({len(plan.get('waypoints', []))} waypoints, "
                f"{len(plan.get('exclusion_zones', []))} exclusions)"
            )

    # ------------------------------------------------------------------ #
    # Plan generation
    # ------------------------------------------------------------------ #
    def _generate_plan(self, command: str) -> dict:
        """Ask Gemini for an intelligent mowing plan, fallback to default."""
        if self.api_available:
            plan = self._ask_gemini(command)
            if plan and 'waypoints' in plan and len(plan['waypoints']) > 0:
                return plan
            self.get_logger().warn("Gemini returned no waypoints — using fallback")

        return self._fallback_plan(command)

    def _ask_gemini(self, command: str) -> dict | None:
        """Send command + camera + context to Gemini, return parsed plan."""
        try:
            from google.genai import types

            # Build image part if available
            parts = []
            if self.latest_frame is not None:
                _, buf = cv2.imencode('.jpg', self.latest_frame,
                                     [cv2.IMWRITE_JPEG_QUALITY, 70])
                parts.append(types.Part.from_bytes(
                    data=buf.tobytes(), mime_type="image/jpeg"))

            # Build context about detected objects
            objects_context = ""
            if self.latest_objects:
                objects_context = (
                    "\n\nThe vision system has previously detected these objects "
                    "in the yard:\n" + json.dumps(self.latest_objects, indent=2)
                )

            prompt = f"""You are the AI brain of an autonomous lawn mower robot.

YARD CONTEXT:
- Fenced yard: 20×20 meters (walls at ±10 m on X and Y axes)
- Safe mowable area: X from {self.yard_x_min + self.safe_margin} to {self.yard_x_max - self.safe_margin}, Y from {self.yard_y_min + self.safe_margin} to {self.yard_y_max - self.safe_margin}
- Known static obstacles: tree at (3,3) radius 0.4m, tree at (-4,2) radius 0.6m, shed at (6,-6) size 3×2m
- Robot starts at origin (0, 0)
- Coordinate system: X = east, Y = north
{objects_context}

VISION AND SEMANTIC MAPPING INSTRUCTIONS:
- Analyze the provided camera image.
- Identify any obstacles or objects on the lawn (e.g., garden gnomes, hoses, toys, pets) and add them to the `exclusion_zones` array.
- The camera is front-facing. Assume any newly identified object in the image is approximately 2.0 meters directly ahead of the robot at its start position (0, 2) since the camera faces forward. So add exclusion zones around (0, 2) for objects found.
- If no image is provided, rely on default exclusions.

USER COMMAND: "{command}"

Generate a mowing plan as a JSON object with EXACTLY these fields:
{{
  "plan_summary": "One-line description of the plan",
  "exclusion_zones": [
    {{"center_x": float, "center_y": float, "radius": float, "reason": "string"}}
  ],
  "waypoints": [
    {{"x": float, "y": float, "yaw": float}}
  ],
  "lane_width": float (spacing between mowing lanes, default 1.0),
  "strategy": "boustrophedon" or "spiral" or "perimeter_first",
  "estimated_time_minutes": int,
  "priority": "thorough" or "quick" or "edges_only"
}}

RULES FOR WAYPOINT GENERATION:
1. Generate a boustrophedon (back-and-forth) pattern that covers the mowable area
2. Respect exclusion zones — waypoints must be at least (radius + 0.5m) away from each exclusion zone center
3. Each waypoint needs x, y coordinates and yaw (heading in radians: 0=east, pi/2=north, pi=west, -pi/2=south)
4. For eastward lanes yaw=0.0, for westward lanes yaw=3.14159
5. Lane endpoints should stay within the safe mowable area
6. If the user says "skip" something, or if you visually identify an obstacle, add it as an exclusion zone
7. If the user says "quick" or "hurry", increase lane_width to 1.5 or 2.0
8. Start from the bottom-left corner and work upward

IMPORTANT: Return ONLY the JSON object, no markdown fences, no extra text."""

            parts.append(types.Part.from_text(text=prompt))

            generate_content_config = types.GenerateContentConfig(
                thinking_config=types.ThinkingConfig(
                    thinking_budget=-1,
                ),
            )

            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[types.Content(role="user", parts=parts)],
                config=generate_content_config,
            )

            raw = response.text.strip()
            # Strip markdown code fences if present
            if raw.startswith("```"):
                lines = raw.split("\n")
                raw = "\n".join(
                    lines[1:-1] if lines[-1].strip() == "```" else lines[1:]
                )

            plan = json.loads(raw)

            # Validate minimum structure
            if 'waypoints' not in plan:
                self.get_logger().warn("Gemini plan missing 'waypoints'")
                return None

            # Ensure yaw values are floats
            for wp in plan['waypoints']:
                wp['x'] = float(wp.get('x', 0))
                wp['y'] = float(wp.get('y', 0))
                wp['yaw'] = float(wp.get('yaw', 0))

            plan.setdefault('exclusion_zones', [])
            plan.setdefault('lane_width', self.default_lane_width)
            plan.setdefault('strategy', 'boustrophedon')
            plan.setdefault('estimated_time_minutes', 25)
            plan.setdefault('priority', 'thorough')
            plan.setdefault('plan_summary', 'AI-generated mowing plan')

            self.get_logger().info(f"Gemini plan: {plan['plan_summary']}")
            return plan

        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Gemini returned invalid JSON: {e}")
        except Exception as e:
            self.get_logger().warn(f"Gemini API call failed: {e}")
        return None

    # ------------------------------------------------------------------ #
    # Fallback planner (no AI)
    # ------------------------------------------------------------------ #
    def _fallback_plan(self, command: str) -> dict:
        """Generate a default boustrophedon plan when Gemini is unavailable."""
        self.get_logger().info("Using fallback boustrophedon planner")

        x_min = self.yard_x_min + self.safe_margin
        x_max = self.yard_x_max - self.safe_margin
        y_min = self.yard_y_min + self.safe_margin
        y_max = self.yard_y_max - self.safe_margin

        # Check for "quick" keywords
        lane_width = self.default_lane_width
        cmd_lower = command.lower()
        if any(w in cmd_lower for w in ['quick', 'fast', 'hurry', 'rain']):
            lane_width = 2.0

        # Known obstacles as default exclusion zones
        exclusions = [
            {"center_x": 3.0, "center_y": 3.0, "radius": 1.0, "reason": "tree"},
            {"center_x": -4.0, "center_y": 2.0, "radius": 1.2, "reason": "tree"},
            {"center_x": 6.0, "center_y": -6.0, "radius": 2.5, "reason": "shed"},
        ]

        # Parse "skip" keywords for additional exclusions
        if 'skip' in cmd_lower:
            # The AI would handle this better, but provide basic support
            pass

        # Generate boustrophedon waypoints
        waypoints = []
        y = y_min
        direction = 1  # 1 = east, -1 = west

        while y <= y_max:
            if direction == 1:
                waypoints.append({"x": x_min, "y": y, "yaw": 0.0})
                waypoints.append({"x": x_max, "y": y, "yaw": 0.0})
            else:
                waypoints.append({"x": x_max, "y": y, "yaw": math.pi})
                waypoints.append({"x": x_min, "y": y, "yaw": math.pi})
            direction *= -1
            y += lane_width

        n_lanes = len(waypoints) // 2
        est_time = int(n_lanes * 2)  # rough estimate: 2 min per lane

        return {
            "plan_summary": f"Fallback boustrophedon: {n_lanes} lanes, "
                            f"lane width {lane_width}m",
            "exclusion_zones": exclusions,
            "waypoints": waypoints,
            "lane_width": lane_width,
            "strategy": "boustrophedon",
            "estimated_time_minutes": est_time,
            "priority": "quick" if lane_width > 1.0 else "thorough",
        }


def main(args=None):
    rclpy.init(args=args)
    node = GeminiMowPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
