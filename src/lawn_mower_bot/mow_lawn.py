#!/usr/bin/env python3
import rclpy
import math
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.parameter import Parameter

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def generate_lawn_mower_path(navigator):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp.sec = 0
    path_msg.header.stamp.nanosec = 0
    
    # SETTINGS FOR FULL COVERAGE
    # We will mow a 5m x 5m patch
    area_width = 5.0   # X direction
    area_height = 5.0  # Y direction
    lane_width = 0.5   # Distance between strips
    resolution = 0.05  # High density waypoints
    
    x, y = 0.0, 0.0
    direction = 1 # 1 for East, -1 for West
    
    num_lanes = int(area_height / lane_width)
    
    # 1. Add Start Point
    start_pose = PoseStamped()
    start_pose.header = path_msg.header
    start_pose.pose.position.x = x
    start_pose.pose.position.y = y
    start_pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, 0.0)
    path_msg.poses.append(start_pose)
    
    for lane in range(num_lanes):
        # --- LONG STRIP (Drive along X) ---
        length_steps = int(area_width / resolution)
        
        for _ in range(length_steps):
            x += (resolution * direction)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            # Face travel direction
            yaw = 0.0 if direction == 1 else 3.14159
            pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, yaw)
            path_msg.poses.append(pose)
            
        # --- U-TURN / SHIFT LANE (Drive along Y) ---
        # Don't shift if it's the last lane
        if lane < num_lanes - 1:
            width_steps = int(lane_width / resolution)
            for _ in range(width_steps):
                y += resolution
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, 1.57) # Face North
                path_msg.poses.append(pose)
            
            # Flip direction for next pass
            direction *= -1

    return path_msg

def main():
    rclpy.init()
    
    nav = BasicNavigator()
    
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])
    
    print("---------------------------------------------------")
    print("Mowing Operation: Lawn Mower Stripes (Boustrophedon)")
    print("Waiting 5 seconds...")
    time.sleep(5.0) 
    print("---------------------------------------------------")
    
    coverage_path = generate_lawn_mower_path(nav)
    
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