"""
hardware_interface.launch.py

Launch file to start the hardware interface for the real robot.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    # Process robot's URDF file
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("robomagellan_description"),
                    "urdf",
                    "robomagellan.urdf.xacro"
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str
    )

    # Use the parsed URDF and publish the robot's state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Load the hardware interfaces via the ros2_control Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("robomagellan_controller"),
                "config",
                "robomagellan_controllers.yaml"
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager
    ])
