#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('pointcloud_topic', '/vision_obstacles')
        
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
            
        self.pc_pub = self.create_publisher(PointCloud2, pc_topic, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Vision Processor Node Started, publishing PointCloud2 to /vision_obstacles.")

        # Camera intrinsics (simplified from 60deg FOV, 640x480)
        self.fov = 1.047198
        self.width = 640
        self.height = 480
        self.focal_length = (self.width / 2) / np.tan(self.fov / 2)
        self.camera_height = 0.22  # camera_link z = wheel_radius(0.07) + base(0.15)
        self.camera_pitch = 0.1   # slight downward pitch in URDF

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Find grass (green areas)
            lower_green = np.array([35, 40, 40])
            upper_green = np.array([85, 255, 255])
            mask_grass = cv2.inRange(hsv, lower_green, upper_green)
            
            # Anything NOT grass is an obstacle
            mask_obstacles = cv2.bitwise_not(mask_grass)
            
            # Focus on the bottom 50% where immediate ground obstacles appear
            mask_obstacles[0:int(self.height * 0.5), :] = 0
            
            # Sparse sampling to avoid massive pointcloud
            y_coords, x_coords = np.nonzero(mask_obstacles[::15, ::15])
            
            points = []
            for y_idx, x_idx in zip(y_coords, x_coords):
                y = y_idx * 15
                x = x_idx * 15
                
                # Pinhole projection to ground plane
                ray_ang_x = np.arctan2(x - self.width / 2.0, self.focal_length)
                ray_ang_y = np.arctan2(y - self.height / 2.0, self.focal_length)
                
                # Project onto ground plane
                tan_val = np.tan(ray_ang_y + self.camera_pitch)
                if tan_val <= 0.01:
                    continue
                ground_dist = self.camera_height / tan_val
                
                if 0.2 < ground_dist < 4.0:
                    px = ground_dist * np.cos(ray_ang_x)
                    py = -ground_dist * np.sin(ray_ang_x)
                    pz = 0.0
                    points.append((px, py, pz))
                    
            # Publish PointCloud2
            pc_msg = self._create_point_cloud(points, msg.header.stamp)
            self.pc_pub.publish(pc_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def _create_point_cloud(self, points, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        
        byte_data = bytearray()
        for p in points:
            byte_data.extend(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = byte_data
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
