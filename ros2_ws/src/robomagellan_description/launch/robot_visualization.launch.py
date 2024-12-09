'''
robot_visualization.launch.py
'''
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

import os
import xacro

def generate_launch_description():
    # Use simulated or real time?
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory(
        'robomagellan_description'))
    xacro_file = os.path.join(pkg_path, 'description',
        'robomagellan.urdf.xacro')

    # Process URDF files with xacro into a single processed URDF file
    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=',
        use_sim_time])

    # Create robot state publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time':
        use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated or real time?'),

        node_robot_state_publisher
    ])
