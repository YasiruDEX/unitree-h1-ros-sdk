"""
Launch file for Unitree H1 robot simulation in Gazebo (Ignition).

This launch file starts:
- Gazebo simulation with H1 world
- robot_state_publisher: Publishes robot TF transforms
- ros_gz_bridge: Bridges Gazebo topics to ROS 2
- rviz2 (optional): Visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_h1_description = get_package_share_directory('h1_description')
    pkg_h1_gazebo = get_package_share_directory('h1_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    urdf_file = os.path.join(pkg_h1_description, 'urdf', 'h1.urdf')
    world_file = os.path.join(pkg_h1_gazebo, 'worlds', 'h1_world.sdf')
    bridge_config = os.path.join(pkg_h1_gazebo, 'config', 'bridge_config.yaml')
    rviz_config = os.path.join(pkg_h1_description, 'rviz', 'h1_display.rviz')
    
    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_h1_description, '..'),
            ':',
            os.path.join(pkg_h1_gazebo, 'worlds')
        ]
    )
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI if true'
    )
    
    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', LaunchConfiguration('world')],
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'h1',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.05',  # Start height (standing position)
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for clock
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher (publishes joint states for TF)
    # This reads from robot_description and publishes joint states
    # Since we're in simulation without real joint feedback yet,
    # we use joint_state_publisher with zeros or gui for manual control
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rate': 50.0,
        }]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        gz_resource_path,
        declare_use_sim_time,
        declare_world,
        declare_rviz,
        declare_gui,
        robot_state_publisher_node,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        joint_state_publisher_node,
        rviz_node,
    ])
