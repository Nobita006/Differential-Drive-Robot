import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
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
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'complex_lawn.world')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mower.rviz')
    cyclonedds_config = os.path.join(pkg_share, 'config', 'cyclonedds.xml')

    # ── FIX: Increase CycloneDDS max participants ──
    # The default limit (~120) is too low for this many nodes.
    set_cyclonedds = SetEnvironmentVariable(
        'CYCLONEDDS_URI', f'file://{cyclonedds_config}'
    )

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
            # Joint States (Gazebo -> ROS)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # LiDAR (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
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

    # 5. Robot Localization (Local EKF only)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 6. SLAM Toolbox (builds map in real-time from LiDAR)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}, {'use_lifecycle_manager': True}],
    )

    # 7. Nav2 Stack
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
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

    # Nav2 + SLAM lifecycle nodes
    # slam_toolbox MUST be first so it activates and starts publishing map→odom
    # before Nav2 servers try to use it
    lifecycle_nodes = ['slam_toolbox', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    # 8. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 9. Coverage Path Planner (BCD on SLAM costmap)
    coverage_planner_node = Node(
        package='lawn_mower_bot',
        executable='coverage_planner.py',
        name='coverage_planner',
        output='screen',
    )

    # 10. Mow Executor (auto-starts mowing)
    gemini_mow_executor_node = Node(
        package='lawn_mower_bot',
        executable='gemini_mow_executor.py',
        name='gemini_mow_executor',
        output='screen',
    )

    return LaunchDescription([
        # Set DDS config FIRST, before any nodes
        set_cyclonedds,
        # Core simulation
        gz_sim_launch,
        bridge_node,
        robot_state_publisher,
        spawn_entity,
        # Localization & SLAM
        ekf_local,
        slam_toolbox_node,
        # Nav2 (delayed to let Gazebo start first)
        TimerAction(period=3.0, actions=[controller_server_node]),
        TimerAction(period=3.0, actions=[planner_server_node]),
        TimerAction(period=3.0, actions=[behavior_server_node]),
        TimerAction(period=3.0, actions=[bt_navigator_node]),
        TimerAction(period=3.0, actions=[waypoint_follower_node]),
        TimerAction(period=10.0, actions=[lifecycle_manager_node]),
        # Coverage planner (needs SLAM map, delayed)
        TimerAction(period=15.0, actions=[coverage_planner_node]),
        # Executor (needs Nav2 active, delayed)
        TimerAction(period=20.0, actions=[gemini_mow_executor_node]),
        # Visualization
        rviz_node,
    ])