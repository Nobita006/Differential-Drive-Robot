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

def generate_straight_line(navigator):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp.sec = 0
    path_msg.header.stamp.nanosec = 0
    
    # Path settings: A simple 3-meter straight line
    total_distance = 3.0 
    resolution = 0.05 # High density (5cm) to keep the robot engaged
    
    steps = int(total_distance / resolution)
    
    for i in range(steps):
        x = i * resolution
        y = 0.0 # Stay on the X-axis
        
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        # Orientation: Always facing Forward (East)
        pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, 0.0)
        
        path_msg.poses.append(pose)
            
    return path_msg

def main():
    rclpy.init()
    
    nav = BasicNavigator()
    
    # Force Sim Time
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])
    
    print("---------------------------------------------------")
    print("Test Mode: Baby Steps")
    print("Goal: Drive 3 meters forward in a straight line.")
    print("Waiting 5 seconds...")
    time.sleep(5.0) 
    print("---------------------------------------------------")
    
    simple_path = generate_straight_line(nav)
    
    print(f"Path generated with {len(simple_path.poses)} waypoints.")
    print("Sending path to robot...")
    
    nav.followPath(simple_path)
    
    i = 0
    while not nav.isTaskComplete():
        i += 1
        feedback = nav.getFeedback()
        if feedback and i % 10 == 0:
            print(f'Distance remaining: {feedback.distance_to_goal:.2f} meters')
            
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Test Complete: Success!')
    elif result == TaskResult.CANCELED:
        print('Test Canceled!')
    elif result == TaskResult.FAILED:
        print('Test Failed!')

    exit(0)

if __name__ == '__main__':
    main()