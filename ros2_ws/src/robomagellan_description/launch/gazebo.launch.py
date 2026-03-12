"""
gazebo.launch.py

Starts the Gazebo simulation and spawns the robot into the world.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    # Directory containing URDF files
    robomagellan_description = get_package_share_directory("robomagellan_description")

    # Parameter specifying the URDF file to load
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        robomagellan_description, "urdf", "robomagellan.urdf.xacro"),
        description="Absolute path to the robot URDF file")

    # Use xacro to parse the specified URDF
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    # Robot state publisher publishes the current state of the robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # Start Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"]),
            launch_arguments=[
                ("gz_args", [" -v 4", " -r", " empty.sdf"])
            ]
    )

    # Spawn the robot into the world
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "robomagellan"]
    )

    """
    Gazebo ROS 2 bridge allows for communications between Gazebo and ROS 2.

    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock maps the Gazebo gz.msgs.Clock
    message to the rosgraph_msgs/msg/Clock topic to allow ROS 2 to receive
    Gazebo simulation time.
    """
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
