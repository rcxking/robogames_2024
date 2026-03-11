"""
display.launch.py

Parses the specified URDF file and displays the robot in Rviz2.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    # Directory containing URDF files
    robomagellan_description_dir = get_package_share_directory("robomagellan_description")

    # Parameter specifying the URDF file to load/display
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        robomagellan_description_dir, "urdf", "robomagellan.urdf.xacro"),
        description="Absolute path to the URDF file to display")

    # Use xacro to parse the specified URDF
    robot_description = ParameterValue(Command(["xacro ",
                                                LaunchConfiguration("model")]),
                                       value_type=str)

    # Robot state publisher publishes the current state of the robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # GUI to allow user to control each wheel with sliders
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # RVIZ
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(robomagellan_description_dir, "rviz", "display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
