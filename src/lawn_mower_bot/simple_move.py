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

def generate_l_shape(navigator):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp.sec = 0
    path_msg.header.stamp.nanosec = 0
    
    resolution = 0.05
    
    # Leg 1: Drive East 2 meters
    for i in range(int(2.0 / resolution)):
        x = i * resolution
        y = 0.0
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, 0.0)
        path_msg.poses.append(pose)
        
    # Corner: Pivot point (2.0, 0.0)
    
    # Leg 2: Drive North 2 meters
    # Note: We start adding Y, keeping X constant
    for i in range(int(2.0 / resolution)):
        x = 2.0
        y = i * resolution
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Orientation: 90 degrees (1.57 radians)
        pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, 1.57)
        path_msg.poses.append(pose)
            
    return path_msg

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    
    print("---------------------------------------------------")
    print("Test Mode: The 'L' Turn")
    print("Goal: Drive 2m, Turn 90 Left, Drive 2m")
    time.sleep(5.0) 
    print("---------------------------------------------------")
    
    l_path = generate_l_shape(nav)
    print(f"Path generated with {len(l_path.poses)} waypoints.")
    nav.followPath(l_path)
    
    i = 0
    while not nav.isTaskComplete():
        i += 1
        feedback = nav.getFeedback()
        if feedback and i % 10 == 0:
            print(f'Distance remaining: {feedback.distance_to_goal:.2f} meters')
            
    if nav.getResult() == TaskResult.SUCCEEDED:
        print('Test Complete: Success!')
    else:
        print('Test Failed!')
    exit(0)

if __name__ == '__main__':
    main()