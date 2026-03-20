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
            get_package_share_directory("robomagellan_firmware"),
            "launch",
            "hardware_interface.launch.py"
        )
    )

    # Load the differential drive controller
    diff_drive_controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robomagellan_controller"),
            "launch",
            "controller.launch.py"
        )
    )

    # Joystick support
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robomagellan_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    return LaunchDescription([
        hw_interface,
        diff_drive_controller,
        joystick
    ])
