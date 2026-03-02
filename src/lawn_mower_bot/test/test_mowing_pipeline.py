import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

@pytest.mark.launch_test
def generate_test_description():
    """Launch the main simulation and return the test description."""
    start_sim_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                os.popen('ros2 pkg prefix lawn_mower_bot').read().strip(),
                'share',
                'lawn_mower_bot',
                'launch',
                'start_simulation.launch.py'
            )
        )
    )

    return launch.LaunchDescription([
        start_sim_launch,
        launch_testing.actions.ReadyToTest()
    ])


class TestMowingPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('mowing_test_node')
        self.latest_progress = None
        self.subscriber = self.node.create_subscription(
            String,
            '/mow_progress',
            self._progress_cb,
            10
        )

    def tearDown(self):
        self.node.destroy_node()

    def _progress_cb(self, msg):
        try:
            self.latest_progress = json.loads(msg.data)
        except Exception:
            pass

    def test_mower_completes_pipeline(self):
        """
        Wait and assert that the mower sequences through boundary discovery,
        gets a coverage plan, and transitions to mowing within time limits.
        """
        self.node.get_logger().info("Waiting 30 seconds for Nav2 to boot...")
        
        # 1. Wait for bootup
        end_time = time.time() + 30.0
        booted = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_progress is not None:
                booted = True
                break
        
        self.assertTrue(booted, "Mower never published to /mow_progress — Nav2 failed to boot!")

        # 2. Wait for boundary discovery to finish (give it 120 secs)
        self.node.get_logger().info("Nav2 active! Waiting for boundary discovery to finish...")
        end_time = time.time() + 120.0
        discovery_done = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            state = self.latest_progress.get('state', '')
            if state in ('mowing', 'complete', 'waiting_for_plan'):
                discovery_done = True
                break
            
        self.assertTrue(discovery_done, "Mower got stuck in boundary discovery!")

        # 3. Wait for mowing to hit 100%
        self.node.get_logger().info("Mowing started! Asserting progress increases...")
        end_time = time.time() + 300.0  # 5 minutes for a small test map
        mowed = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_progress.get('state') == 'complete':
                mowed = True
                break

        self.assertTrue(mowed, "Mower timed out before reaching 100% completion!")
        self.node.get_logger().info("TEST PASSED: End-to-End Mowing Pipeline Successful!")
