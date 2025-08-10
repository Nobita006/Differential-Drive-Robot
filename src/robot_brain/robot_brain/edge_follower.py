import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class EdgeFollowerNode(Node):
    def __init__(self):
        super().__init__('edge_follower_node')
        self.get_logger().info("Edge Follower Node has started.")

        # Subscriber to the camera feed
        self.camera_subscriber = self.create_subscription(
            Image,
            '/my_robot/camera/image_raw', # This is the topic the virtual camera publishes to
            self.image_callback,
            10)

        # Publisher for robot movement commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        """This function is called every time a new image is received."""
        self.get_logger().info('Received a new image!')

        # ===============================================
        #  <<< YOUR AI/COMPUTER VISION LOGIC GOES HERE >>>
        #
        # 1. Convert the ROS Image message to an OpenCV image.
        # 2. Process the image with your semantic segmentation model.
        # 3. Find the "edge" between grass and pavement.
        # 4. Decide if you need to turn left, right, or go straight.
        #
        # ===============================================

        # For now, let's just make the robot move forward slowly.
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        move_cmd.angular.z = 0.0 # No turning

        # Publish the movement command
        self.velocity_publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = EdgeFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()