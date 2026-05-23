"""
odometry_motion_model.launch.py

Starts the node implementing the odometry motion model for a planar robot.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Spawn odometry motion model
    odometry_motion_model = Node(
        package="robomagellan_localization",
        executable="odometry_motion_model",
        name="odometry_motion_model",
        output="screen"
    )

    return LaunchDescription([
        odometry_motion_model
    ])
