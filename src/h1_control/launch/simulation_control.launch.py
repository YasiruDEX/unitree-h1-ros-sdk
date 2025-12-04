"""
Combined Launch File for H1 Gazebo Simulation with Control

This launch file starts:
- Gazebo simulation with H1 robot
- High-level control node
- Gazebo simulation controller (bridges cmd_vel to joint commands)
- Keyboard teleoperation (optional)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_h1_gazebo = get_package_share_directory('h1_gazebo')
    pkg_h1_control = get_package_share_directory('h1_control')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    declare_teleop = DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Launch keyboard teleop in separate terminal'
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_h1_gazebo, 'launch', 'h1_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # High-level control node
    high_level_control_node = Node(
        package='h1_control',
        executable='high_level_control.py',
        name='h1_high_level_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Gazebo simulation controller - delayed start to wait for Gazebo
    gazebo_sim_controller_node = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to start
        actions=[
            Node(
                package='h1_control',
                executable='gazebo_sim_controller.py',
                name='gazebo_sim_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'control_rate': 100.0,
                }]
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_teleop,
        gazebo_launch,
        high_level_control_node,
        gazebo_sim_controller_node,
    ])
