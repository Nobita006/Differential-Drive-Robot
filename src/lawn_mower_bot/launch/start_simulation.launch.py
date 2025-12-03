import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('lawn_mower_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'mower.urdf.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'lawn.world')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mower.rviz')
    map_file = os.path.join(pkg_share, 'maps', 'lawn_map.yaml')

    # 1. Gazebo Simulation
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. ROS-Gazebo Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Cmd Vel (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odom (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # IMU (Gazebo -> ROS)
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # GPS (Gazebo -> ROS)
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # Joint States (Gazebo -> ROS) - CRITICAL FOR WHEELS
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # TF (Gazebo -> ROS) - Helper for ground truth
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # 3. Robot State Publisher
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
    )

    # 4. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'lawn_mower', '-topic', 'robot_description', '-z', '0.2'],
        output='screen'
    )

    # 5. Robot Localization (Dual EKF)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[('imu', 'imu/data'),
                    ('gps/fix', 'gps/fix'), 
                    ('odometry/gps', '/odometry/gps'),
                    ('odometry/filtered', 'odometry/global')]
    )

    # 6. NAVIGATION NODES
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_file}, {'use_sim_time': True}]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        # remappings=[('cmd_vel', 'cmd_vel_nav')] 
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )
    
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    lifecycle_nodes = ['map_server', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    # 7. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gz_sim_launch,
        bridge_node, # Added Bridge here
        robot_state_publisher,
        spawn_entity,
        ekf_local,
        ekf_global,
        navsat_transform,
        map_server_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        TimerAction(period=3.0, actions=[lifecycle_manager_node]),
        rviz_node
    ])