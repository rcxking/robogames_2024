"""
sim.launch.py

Starts Gazebo and spawns the simulated robot.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

import os

def generate_launch_description():
    # Spawn Gazebo and the simulated robot
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robomagellan_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    # Start the differential drive controller
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robomagellan_controller"),
            "launch",
            "controller.launch.py"
        )
    )

    return LaunchDescription([
        gazebo,
        controller
    ])
