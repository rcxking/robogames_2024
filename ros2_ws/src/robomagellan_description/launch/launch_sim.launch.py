from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Include robot_state_publisher launch file
    package_name = 'robomagellan_description'

    rv = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch',
                'robot_visualization.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include Gazebo launch file from package gazebo_ros
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch',
                'gazebo.launch.py')])
    )

    # Spawn an instance of the simulated robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robot'],
            output='screen')

    # ROS 2 Controls - Differential Drive Controller
    diff_drive_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont']
    )

    # ROS 2 Controls - Joint State Broadcaster
    joint_broad_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad']
    )

    return LaunchDescription([
        rv,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner
    ])
