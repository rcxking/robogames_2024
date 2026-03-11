"""
hardware_interface.launch.py

Launch file to load and activate the hardware interfaces for the RoboMagellan
robot.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

import os

# "main" function for launch files
def generate_launch_description():
    # Parse and publish the robot's URDF with xacro
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("robomagellan_description"),
                    "urdf",
                    "robomagellan.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    return LaunchDescription([
    ])
