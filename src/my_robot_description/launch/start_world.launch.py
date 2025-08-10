import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    my_robot_description_pkg = get_package_share_directory("my_robot_description")
    world_file = os.path.join(my_robot_description_pkg, "worlds", "my_world.sdf")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        my_robot_description_pkg, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r ", world_file]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot",
                   "-z", "0.1"],
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen"
    )
    
    # --- THIS IS THE NEW, ROBUST FIX ---
    # A timer action that waits for 2 seconds before executing the unpause command
    unpause_action = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/world/default/control', 'gz.msgs.WorldControl', '{pause: false}'],
                shell=True,
                output='screen'
            )
        ]
    )
    # --- END OF FIX ---
    
    follower_node = Node(
        package='robot_brain',
        executable='follower',
        name='edge_follower_node'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gz_ros2_bridge,
        gazebo,
        gz_spawn_entity,
        unpause_action,  # Add the unpause action to the launch description
        follower_node,
    ])