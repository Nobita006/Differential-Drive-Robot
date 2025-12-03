#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path  # <--- NEW IMPORT
import time

def generate_spiral_path(navigator):
    """Generates a simple square spiral path coverage pattern."""
    
    # --- Create the Path Message Object ---
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = navigator.get_clock().now().to_msg()
    # --------------------------------------
    
    # --- CONFIGURATION ---
    spacing = 0.3      # Distance between spiral lines (meters)
    max_radius = 8.0   # How big the area is (meters)
    # ---------------------
    
    # Start at the center (relative to map origin)
    x, y = 0.0, 0.0
    dx, dy = spacing, 0.0
    segment_length = spacing
    current_len = 0
    
    # Calculate approx number of points needed
    total_points = int((max_radius * 2 / spacing) ** 2)
    
    print(f"Calculating spiral path with {total_points} points...")

    for _ in range(total_points):
        # Create a waypoint
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = path_msg.header.stamp # Use same stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Orientation (Simplified: just facing forward for now)
        pose.pose.orientation.w = 1.0 
        pose.pose.orientation.z = 0.0
        
        # Add to the Path object, not a list
        path_msg.poses.append(pose)
        
        # Advance coordinates
        x += dx
        y += dy
        current_len += spacing
        
        # Logic to turn 90 degrees after finishing a segment
        if current_len >= segment_length:
            current_len = 0
            # Rotate direction: (dx, dy) -> (-dy, dx)
            dx, dy = -dy, dx
            # Every two turns, the segment length increases
            if dy == 0:
                segment_length += spacing

        # Stop if we hit the boundary
        if abs(x) > max_radius or abs(y) > max_radius:
            break
            
    return path_msg

def main():
    rclpy.init()
    
    # Initialize the Navigator
    nav = BasicNavigator()
    
    print("---------------------------------------------------")
    print("ASSUMPTION: Simulation and Nav2 are ALREADY ACTIVE.")
    print("Sending path in 3 seconds...")
    print("---------------------------------------------------")
    time.sleep(3.0) 
    
    # Generate the path
    print("Generating coverage path...")
    coverage_path = generate_spiral_path(nav)
    
    print(f"Path generated. Sending {len(coverage_path.poses)} waypoints to robot...")
    
    # Command the robot to follow the path
    # Now passing a proper 'Path' object, so it won't crash!
    nav.followPath(coverage_path)
    
    # Monitor progress
    i = 0
    while not nav.isTaskComplete():
        i += 1
        feedback = nav.getFeedback()
        if feedback and i % 10 == 0:
            print(f'Distance remaining: {feedback.distance_to_goal:.2f} meters')
            
    # Print Result
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