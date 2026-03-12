"""
competition.launch.py

Launch file for the real robot to be used during a RoboMagellan competition.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

import os

# "main" function for launch files
def generate_launch_description():
    # Load the robot's hardware interfaces
    hw_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robomagellan_hw_interface"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    return LaunchDescription([
        hw_interface
    ])
