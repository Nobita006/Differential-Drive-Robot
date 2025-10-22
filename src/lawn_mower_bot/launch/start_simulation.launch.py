import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('lawn_mower_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Paths to config files
    urdf_file = os.path.join(pkg_share, 'urdf', 'mower.urdf.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'lawn.world')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mower.rviz')

    # Gazebo launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Process the URDF file with xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
    )
    
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'lawn_mower', '-topic', 'robot_description', '-z', '0.1'],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaw_offset': 1.5707963},
                    {'magnetic_declination_radians': 0.0},
                    {'delay': 0.0},
                    {'broadcast_cartesian_transform': True},
                    {'publish_filtered_gps': True}],
        remappings=[('imu', 'imu/data'),
                    ('gps/fix', 'gps/fix'),
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/filtered', '/odometry/global')]
    )
    
    # Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(pkg_share, 'maps', 'lawn_map.yaml'),
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'use_collision_monitor': 'False',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # We do not need to set any environment variables.
        # The modern system handles this automatically.
        gz_sim_launch,
        robot_state_publisher_node,
        spawn_node,
        robot_localization_node,
        navsat_transform_node,
        nav2_bringup_launch,
        rviz_node
    ])