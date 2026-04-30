#!/usr/bin/env python3
import sys
import os
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import MapMetaData

# Add the src dir to path so we can import the planner directly
sys.path.append(os.path.join(os.path.dirname(__file__), 'src', 'lawn_mower_bot'))
from coverage_planner import CoveragePlanner

def main():
    rclpy.init()
    
    # Initialize the planner without starting ROS spinners
    planner = CoveragePlanner()
    
    # Set parameters for the test
    planner.lane_width = 0.5
    planner.robot_radius = 0.25
    
    # Create a dummy map (20m x 20m)
    width, height = 400, 400
    planner.map_info = MapMetaData()
    planner.map_info.resolution = 0.05
    planner.map_info.origin.position.x = 0.0
    planner.map_info.origin.position.y = 0.0
    planner.map_info.width = width
    planner.map_info.height = height
    
    # Create a free space mask (True = free, False = obstacle)
    free_mask = np.ones((height, width), dtype=bool)
    
    # Add a pool / obstacle in the middle
    free_mask[150:250, 100:200] = False
    
    # Add a weird corner obstacle
    free_mask[300:350, 300:350] = False
    
    print("Generating coverage waypoints...")
    waypoints = planner._boustrophedon_on_free_space(free_mask)
    print(f"Generated {len(waypoints)} waypoints.")
    
    # Extract X and Y for plotting
    x_coords = [wp['x'] for wp in waypoints]
    y_coords = [wp['y'] for wp in waypoints]
    
    # Plotting
    plt.figure(figsize=(10, 10))
    plt.title("Coverage Path Planner Visualizer")
    
    # Draw obstacles
    plt.imshow(free_mask.T, origin='lower', extent=[0, 20, 0, 20], cmap='gray', alpha=0.3)
    
    # Draw path
    plt.plot(x_coords, y_coords, '-o', markersize=3, color='blue', label='Path')
    
    # Draw start/end
    if waypoints:
        plt.plot(x_coords[0], y_coords[0], 'go', markersize=8, label='Start')
        plt.plot(x_coords[-1], y_coords[-1], 'ro', markersize=8, label='End')
        
    plt.legend()
    plt.grid(True)
    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    
    # Save the plot
    save_path = '/home/sayan/ros2_ws/coverage_plot.png'
    plt.savefig(save_path)
    print(f"Plot saved to {save_path}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
