#!/usr/bin/env python3
import rclpy
import math
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.parameter import Parameter

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def generate_spiral_path(navigator):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = navigator.get_clock().now().to_msg()
    
    spacing = 0.5 
    max_radius = 4.0 
    
    x, y = 0.0, 0.0 
    dx, dy = spacing, 0.0 
    segment_length = spacing
    current_len = 0
    
    total_points = int((max_radius * 2 / spacing) ** 2)
    
    for _ in range(total_points):
        # FIX: Move FIRST, then append. 
        # This prevents the first point from being (0,0)
        x += dx
        y += dy
        current_len += spacing
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = path_msg.header.stamp 
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        # Calculate Yaw
        yaw = math.atan2(dy, dx)
        pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, yaw)
        
        path_msg.poses.append(pose)
        
        # Spiral logic
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
    
    # Force Sim Time (Critical for Gazebo synchronization)
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])
    
    print("---------------------------------------------------")
    print("Nav2 Active. Waiting 5 seconds for systems to settle...")
    time.sleep(5.0) 
    print("---------------------------------------------------")
    
    # Reset costmaps (Optional, clears ghosts)
    # nav.clearAllCostmaps() 
    
    print("Generating coverage path...")
    coverage_path = generate_spiral_path(nav)
    
    print(f"Path generated with {len(coverage_path.poses)} waypoints.")
    print("Sending path to robot...")
    
    # Execute the path
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