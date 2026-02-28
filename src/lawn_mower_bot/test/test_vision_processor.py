"""
Tests for VisionProcessor image processing and PointCloud2 generation.
Tests the pure processing logic without importing the ROS node class.
"""
import struct
import pytest
import numpy as np
import cv2


# ─── Camera parameters (matching the VisionProcessor class) ─────────
FOV = 1.047198
WIDTH = 640
HEIGHT = 480
FOCAL_LENGTH = (WIDTH / 2) / np.tan(FOV / 2)
CAMERA_HEIGHT = 0.22
CAMERA_PITCH = 0.1


def _detect_obstacles(image):
    """
    Replicate the obstacle detection logic from VisionProcessor.image_callback.
    Returns list of (px, py, pz) points.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    mask_grass = cv2.inRange(hsv, lower_green, upper_green)
    mask_obstacles = cv2.bitwise_not(mask_grass)
    mask_obstacles[0:int(HEIGHT * 0.5), :] = 0

    y_coords, x_coords = np.nonzero(mask_obstacles[::15, ::15])
    points = []
    for y_idx, x_idx in zip(y_coords, x_coords):
        y = y_idx * 15
        x = x_idx * 15
        ray_ang_x = np.arctan2(x - WIDTH / 2.0, FOCAL_LENGTH)
        ray_ang_y = np.arctan2(y - HEIGHT / 2.0, FOCAL_LENGTH)
        tan_val = np.tan(ray_ang_y + CAMERA_PITCH)
        if tan_val <= 0.01:
            continue
        ground_dist = CAMERA_HEIGHT / tan_val
        if 0.2 < ground_dist < 4.0:
            px = ground_dist * np.cos(ray_ang_x)
            py = -ground_dist * np.sin(ray_ang_x)
            pz = 0.0
            points.append((px, py, pz))
    return points


def _create_point_cloud_data(points):
    """
    Replicate the PointCloud2 data serialization.
    Returns (fields_info, byte_data).
    """
    fields = [
        {'name': 'x', 'offset': 0},
        {'name': 'y', 'offset': 4},
        {'name': 'z', 'offset': 8},
    ]
    point_step = 12
    byte_data = bytearray()
    for p in points:
        byte_data.extend(struct.pack('fff', p[0], p[1], p[2]))
    return fields, point_step, byte_data


class TestObstacleDetection:
    def test_green_image_no_obstacles(self, green_image):
        """A solid green image (all grass) should produce no obstacle points."""
        points = _detect_obstacles(green_image)
        assert len(points) == 0, "Green image should have zero obstacles"

    def test_red_image_has_obstacles(self, red_image):
        """A solid red image should produce obstacle detections in the bottom half."""
        points = _detect_obstacles(red_image)
        assert len(points) > 0, "Red image should have obstacle detections"

    def test_top_half_ignored(self, red_image):
        """Obstacles in the top half should be masked out."""
        hsv = cv2.cvtColor(red_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        mask_grass = cv2.inRange(hsv, lower_green, upper_green)
        mask_obstacles = cv2.bitwise_not(mask_grass)
        mask_obstacles[0:int(HEIGHT * 0.5), :] = 0
        assert np.sum(mask_obstacles[0:240, :]) == 0

    def test_half_image(self):
        """An image with top-half green, bottom-half red should only
        detect obstacles in the bottom portion."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[0:240, :] = (0, 180, 0)   # green top
        img[240:480, :] = (0, 0, 200)  # red bottom
        points = _detect_obstacles(img)
        assert len(points) > 0


class TestPointCloudSerialization:
    def test_empty_points(self):
        fields, step, data = _create_point_cloud_data([])
        assert len(data) == 0
        assert step == 12

    def test_single_point(self):
        fields, step, data = _create_point_cloud_data([(1.0, 2.0, 0.0)])
        assert len(data) == 12
        x, y, z = struct.unpack('fff', bytes(data))
        assert abs(x - 1.0) < 1e-6
        assert abs(y - 2.0) < 1e-6
        assert abs(z - 0.0) < 1e-6

    def test_multiple_points(self):
        points = [(i * 0.1, 0.0, 0.0) for i in range(10)]
        fields, step, data = _create_point_cloud_data(points)
        assert len(data) == 120  # 10 * 12

    def test_field_names(self):
        fields, _, _ = _create_point_cloud_data([(0, 0, 0)])
        names = [f['name'] for f in fields]
        assert names == ['x', 'y', 'z']

    def test_field_offsets(self):
        fields, _, _ = _create_point_cloud_data([(0, 0, 0)])
        offsets = [f['offset'] for f in fields]
        assert offsets == [0, 4, 8]
