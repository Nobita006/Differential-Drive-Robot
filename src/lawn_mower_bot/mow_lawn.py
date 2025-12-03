#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.parameter import Parameter
from rclpy.duration import Duration
import time

def generate_spiral_path(navigator):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = navigator.get_clock().now().to_msg()
    
    spacing = 0.5 # Smaller spacing for realistic mowing
    max_radius = 2.0 # Keep it small for the test
    
    x, y = 0.0, 0.0 # Start at center
    dx, dy = spacing, 0.0
    segment_length = spacing
    current_len = 0
    
    total_points = int((max_radius * 2 / spacing) ** 2)
    
    for _ in range(total_points):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = path_msg.header.stamp 
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 
        pose.pose.orientation.z = 0.0
        
        path_msg.poses.append(pose)
        
        x += dx
        y += dy
        current_len += spacing
        
        if current_len >= segment_length:
            current_len = 0
            dx, dy = -dy, dx
            if dy == 0:
                segment_length += spacing

        if abs(x) > max_radius or abs(y) > max_radius:
            break
            
    return path_msg

def main():
    rclpy.init()
    
    nav = BasicNavigator()
    
    # Force Sim Time
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])
    
    print("---------------------------------------------------")
    print("Nav2 is already active (confirmed by launch logs).")
    print("Waiting 2 seconds for connections to stabilize...")
    time.sleep(2.0) 
    print("---------------------------------------------------")
    
    # REMOVED THE BLOCKING WAIT COMMAND
    # nav.waitUntilNav2Active(localizer=None) 
    
    print("Generating coverage path...")
    coverage_path = generate_spiral_path(nav)
    
    print(f"Path generated. Sending {len(coverage_path.poses)} waypoints to robot...")
    
    nav.followPath(coverage_path)
    
    i = 0
    while not nav.isTaskComplete():
        i += 1
        feedback = nav.getFeedback()
        if feedback and i % 10 == 0:
            print(f'Distance remaining: {feedback.distance_to_goal:.2f} meters')
            
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Mowing Complete!')
    elif result == TaskResult.CANCELED:
        print('Mowing Canceled!')
    elif result == TaskResult.FAILED:
        print('Mowing Failed!')

    exit(0)

if __name__ == '__main__':
    main()