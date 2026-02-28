#!/usr/bin/env python3
"""
Gemini Robotics-ER Scene Analyzer for Autonomous Lawn Mower.

This node periodically captures camera frames and sends them to the
Gemini Robotics-ER API for high-level scene understanding. It publishes:
  - /gemini/scene_report (String): Human-readable scene analysis
  - /gemini/hazards (String): JSON list of detected hazards with positions

This acts as the "heavy thinker" — it doesn't control the robot directly,
but provides intelligence that can be used for decision-making.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
import base64
import os

class GeminiPlanner(Node):
    def __init__(self):
        super().__init__('gemini_planner')

        # Parameters
        self.declare_parameter('api_key', '')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('analysis_interval', 5.0)  # seconds between API calls

        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.analysis_interval = self.get_parameter('analysis_interval').get_parameter_value().double_value

        if not self.api_key:
            self.api_key = os.environ.get('GEMINI_API_KEY', '')

        if not self.api_key:
            self.get_logger().error("No Gemini API key provided! Set 'api_key' param or GEMINI_API_KEY env var.")
            self.api_available = False
        else:
            self.api_available = True
            try:
                from google import genai
                self.client = genai.Client(api_key=self.api_key)
                self.model_name = "gemini-2.0-flash"
                self.get_logger().info(f"Gemini API initialized with model: {self.model_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize Gemini API: {e}")
                self.api_available = False

        # Subscribers
        self.bridge = CvBridge()
        self.latest_frame = None
        self.subscription = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        # Publishers
        self.scene_pub = self.create_publisher(String, '/gemini/scene_report', 10)
        self.hazard_pub = self.create_publisher(String, '/gemini/hazards', 10)
        self.objects_pub = self.create_publisher(String, '/gemini/objects', 10)

        # Timer for periodic analysis
        self.last_analysis_time = 0.0
        self.timer = self.create_timer(self.analysis_interval, self.analyze_scene)

        self.get_logger().info("Gemini Planner Node Started.")

    def image_callback(self, msg):
        """Store the latest camera frame."""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def analyze_scene(self):
        """Send frame to Gemini for scene analysis."""
        if not self.api_available or self.latest_frame is None:
            return

        try:
            from google.genai import types

            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', self.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            image_bytes = buffer.tobytes()

            prompt = """You are the AI brain of an autonomous lawn mower robot.
Analyze this camera image from the robot's front-facing camera.

Provide a JSON response with exactly these fields:
{
  "scene_summary": "Brief description of what you see",
  "terrain": "tall_grass/short_grass/dirt/pavement/mixed",
  "obstacles": [
    {"type": "tree/rock/person/animal/furniture/flower_bed/toy/garden_hose/sprinkler/unknown", "position": "left/center/right", "distance": "near/medium/far", "danger_level": "low/medium/high"}
  ],
  "detected_objects": [
    {"label": "semantic name of the object", "category": "vegetation/structure/living/item/terrain_feature", "position": "left/center/right", "should_avoid": true/false, "reason": "why to avoid or not"}
  ],
  "terrain_zones": [
    {"type": "tall_grass/short_grass/dirt/flower_bed/paved", "position": "left/center/right", "coverage": "small/medium/large"}
  ],
  "mowing_quality": "good/needs_attention/obstacle_blocking",
  "recommended_action": "continue/slow_down/stop/turn_left/turn_right",
  "confidence": 0.0 to 1.0
}

IMPORTANT:
- Identify WHAT objects are (e.g. "rose bush" not just "obstacle")
- Classify terrain (tall grass needs mowing, short grass already done)
- Flag items that should NOT be mowed over (flower beds, garden decorations, toys)
- Be concise. Only report items actually visible."""

            # Call Gemini API
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[
                    types.Content(
                        role="user",
                        parts=[
                            types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
                            types.Part.from_text(text=prompt),
                        ],
                    )
                ],
            )

            response_text = response.text.strip()

            # Try to parse JSON from response
            # Strip markdown code fences if present
            clean_text = response_text
            if clean_text.startswith("```"):
                lines = clean_text.split("\n")
                clean_text = "\n".join(lines[1:-1] if lines[-1].strip() == "```" else lines[1:])

            try:
                analysis = json.loads(clean_text)
            except json.JSONDecodeError:
                analysis = {"scene_summary": response_text, "raw": True}

            # Publish scene report
            scene_msg = String()
            scene_msg.data = analysis.get("scene_summary", "Analysis complete")
            self.scene_pub.publish(scene_msg)

            # Publish hazards
            hazards = analysis.get("obstacles", [])
            high_danger = [h for h in hazards if h.get("danger_level") == "high"]

            hazard_msg = String()
            hazard_msg.data = json.dumps({
                "obstacles": hazards,
                "high_danger_count": len(high_danger),
                "recommended_action": analysis.get("recommended_action", "continue"),
                "mowing_quality": analysis.get("mowing_quality", "unknown"),
            })
            self.hazard_pub.publish(hazard_msg)

            # Publish semantic objects for the mow planner
            detected_objects = analysis.get("detected_objects", [])
            terrain_zones = analysis.get("terrain_zones", [])
            if detected_objects or terrain_zones:
                objects_msg = String()
                objects_msg.data = json.dumps({
                    "detected_objects": detected_objects,
                    "terrain_zones": terrain_zones,
                    "terrain": analysis.get("terrain", "unknown"),
                })
                self.objects_pub.publish(objects_msg)

            # Log summary
            action = analysis.get("recommended_action", "unknown")
            n_obs = len(hazards)
            summary = analysis.get("scene_summary", "")[:80]
            self.get_logger().info(
                f"[Gemini] {summary} | Obstacles: {n_obs} | Action: {action}")

        except Exception as e:
            self.get_logger().warn(f"Gemini analysis failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GeminiPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
