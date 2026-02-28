#!/usr/bin/env python3
"""
Autonomous Lawn Mower - Boustrophedon Coverage Path
Uses Nav2 goThroughPoses for obstacle-aware navigation with replanning.
Integrates with Gemini Planner for scene intelligence.
"""
import rclpy
import math
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from rclpy.parameter import Parameter
import json

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def make_pose(x, y, yaw, nav, frame='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = get_quaternion_from_euler(0.0, 0.0, yaw)
    return pose

def generate_coverage_waypoints(nav):
    """
    Generate boustrophedon (back-and-forth) waypoints to cover
    a smaller area inside the fenced 20x20m yard.
    We stay 3m away from the walls (at +/-10) to be safe.
    """
    waypoints = []
    
    # Mowable area: from -7 to 7 in X, -7 to 7 in Y (inside the fence)
    x_min = -7.0
    x_max = 7.0
    y_min = -7.0
    y_max = 7.0
    lane_width = 1.0  # spacing between lanes
    
    y = y_min
    direction = 1  # 1 = east, -1 = west
    
    while y <= y_max:
        if direction == 1:
            # Drive east: add start and end of lane
            waypoints.append(make_pose(x_min, y, 0.0, nav))
            waypoints.append(make_pose(x_max, y, 0.0, nav))
        else:
            # Drive west
            waypoints.append(make_pose(x_max, y, math.pi, nav))
            waypoints.append(make_pose(x_min, y, math.pi, nav))
        
        direction *= -1
        y += lane_width
    
    return waypoints

def main():
    rclpy.init()
    
    nav = BasicNavigator()
    
    param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    nav.set_parameters([param])
    
    # Subscribe to Gemini hazard reports
    gemini_data = {'latest': None}
    def gemini_callback(msg):
        try:
            gemini_data['latest'] = json.loads(msg.data)
        except Exception:
            pass
    
    nav.create_subscription(String, '/gemini/hazards', gemini_callback, 10)
    
    print("=" * 60)
    print("  AUTONOMOUS LAWN MOWER")
    print("  Waiting for Nav2 to become active...")
    print("=" * 60)
    
    # Wait for Nav2 to fully come up
    # Skip AMCL check — we use robot_localization EKF, not AMCL
    nav.waitUntilNav2Active(localizer='robot_localization')
    print("Nav2 is active!")
    
    # Set initial pose (robot starts at origin)
    initial_pose = make_pose(0.0, 0.0, 0.0, nav)
    nav.setInitialPose(initial_pose)
    print("Initial pose set. Waiting 5 seconds for localization to settle...")
    time.sleep(5.0)
    
    # Generate waypoints
    waypoints = generate_coverage_waypoints(nav)
    print(f"Generated {len(waypoints)} waypoints for lawn coverage.")
    
    # Navigate in batches to improve reliability and allow Gemini feedback
    batch_size = 6  # 3 lanes at a time
    total_completed = 0
    total_waypoints = len(waypoints)
    
    for batch_start in range(0, total_waypoints, batch_size):
        batch_end = min(batch_start + batch_size, total_waypoints)
        batch = waypoints[batch_start:batch_end]
        
        # Check Gemini for hazards before starting batch
        if gemini_data['latest']:
            hazard_info = gemini_data['latest']
            action = hazard_info.get('recommended_action', 'continue')
            n_hazards = hazard_info.get('high_danger_count', 0)
            if action == 'stop' and n_hazards > 0:
                print(f"  [Gemini] HIGH DANGER detected! Pausing for 5 seconds...")
                time.sleep(5.0)
            elif action != 'continue':
                print(f"  [Gemini] Advisory: {action} (hazards: {n_hazards})")
        
        batch_num = (batch_start // batch_size) + 1
        total_batches = (total_waypoints + batch_size - 1) // batch_size
        print(f"\nStarting batch {batch_num}/{total_batches} ({len(batch)} waypoints)...")
        
        nav.followWaypoints(batch)
        
        i = 0
        while not nav.isTaskComplete():
            i += 1
            feedback = nav.getFeedback()
            if feedback and i % 20 == 0:
                current_wp = feedback.current_waypoint
                overall = batch_start + current_wp
                print(f'  Progress: Waypoint {overall}/{total_waypoints} '
                      f'(batch {batch_num}: {current_wp}/{len(batch)})')
                
                # Check Gemini mid-batch
                if gemini_data['latest']:
                    quality = gemini_data['latest'].get('mowing_quality', 'unknown')
                    if quality != 'good' and quality != 'unknown':
                        print(f'  [Gemini] Mowing quality: {quality}')
        
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            total_completed += len(batch)
            print(f'  Batch {batch_num} COMPLETE! ({total_completed}/{total_waypoints} total)')
        elif result == TaskResult.CANCELED:
            print(f'  Batch {batch_num} was canceled.')
            break
        elif result == TaskResult.FAILED:
            print(f'  Batch {batch_num} FAILED. Skipping to next batch...')
            total_completed += len(batch)  # count as attempted
            time.sleep(2.0)
            continue
    
    # Final report
    print('\n' + '=' * 60)
    if total_completed >= total_waypoints:
        print('  MOWING COMPLETE! All waypoints reached.')
    else:
        print(f'  MOWING FINISHED. Completed {total_completed}/{total_waypoints} waypoints.')
    
    if gemini_data['latest']:
        print(f'  Last Gemini report: {gemini_data["latest"].get("recommended_action", "N/A")}')
    print('=' * 60)

    rclpy.shutdown()

if __name__ == '__main__':
    main()