"""
controller.launch.py

Spawns the differential drive controller.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joint state broadcaster reads the states from all state interfaces and
    # publishes states on /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Start the differential drive controller
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robomagellan_controller",
                   "--controller-manager",
                   "/controller_manager"
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        wheel_controller_spawner
    ])
