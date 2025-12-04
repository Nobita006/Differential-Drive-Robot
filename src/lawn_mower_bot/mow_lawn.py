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

def generate_dense_spiral(navigator):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp.sec = 0
    path_msg.header.stamp.nanosec = 0
    
    # SETTINGS MATCHING THE SUCCESSFUL 'L' TEST
    spacing = 1.0 
    max_radius = 8.0 
    resolution = 0.05 # 5cm resolution (CRITICAL for smooth turns)
    
    x, y = 0.0, 0.0 
    dx, dy = spacing, 0.0 
    segment_length = spacing
    current_len = 0
    
    # 1. Add Start Point
    start_pose = PoseStamped()
    start_pose.header = path_msg.header
    start_pose.pose.position.x = x
    start_pose.pose.position.y = y
    start_pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, 0.0)
    path_msg.poses.append(start_pose)
    
    # 2. Generate Spiral
    total_legs = int(max_radius * 4) # Estimate number of turns
    
    for _ in range(total_legs):
        # Determine number of small steps for this segment
        steps = int(segment_length / resolution)
        
        # Calculate the small increment per step
        step_x = dx / steps if steps > 0 else 0
        step_y = dy / steps if steps > 0 else 0
        
        # Interpolate points along the segment
        for _ in range(steps):
            x += step_x
            y += step_y
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            
            # Calculate Yaw based on current direction
            yaw = math.atan2(dy, dx)
            pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, yaw)
            
            path_msg.poses.append(pose)
            
        # Spiral logic (Turn corner)
        current_len += spacing 
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
    print("Mowing Operation: Full Spiral")
    print("Strategy: High-Density Waypoints (5cm)")
    print("Waiting 5 seconds...")
    time.sleep(5.0) 
    print("---------------------------------------------------")
    
    coverage_path = generate_dense_spiral(nav)
    
    print(f"Path generated with {len(coverage_path.poses)} waypoints.")
    print("Sending path to robot...")
    
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