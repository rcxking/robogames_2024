"""
sim.launch.py

Starts Gazebo and spawns the simulated robot.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

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

    # Start the joystick teleop nodes.  TODO: Use keyboard teleop node instead?
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robomagellan_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # Spawn Rviz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("robomagellan_description"),
            "rviz", "sim.rviz")]
    )

    return LaunchDescription([
        gazebo,
        controller,
        joystick,
        rviz
    ])
