"""
joystick_teleop.launch.py

Launches the nodes to enable sending robot velocity commands from a joystick
(USB or Bluetooth).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Use simulated or real time?
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time",
                                             default_value="True",
                                             description="Use simulated time?"
    )

    # The joy node connects to the joystick at the specified interface and
    # publishes the joystick commands to the /joy topic.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("robomagellan_controller"),
                         "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # The joy_teleop node subscribes to the /joy topic and maps those values to
    # the robot's velocity inputs.
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(get_package_share_directory("robomagellan_controller"),
                         "config", "joy_teleop.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop
    ])
