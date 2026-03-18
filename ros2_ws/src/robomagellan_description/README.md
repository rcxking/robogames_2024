# robomagellan_description

This package contains the robot's URDF files to support Gazebo/ROS2 control and
launch files to view the URDF and start Gazebo as needed.

## URDF Files

The following is a list of the URDF files and their purpose:

1. `robomagellan.urdf.xacro`: The main robot URDF
2. `gazebo.xacro`: Gazebo-specific parameters
3. `ros2_control.xacro`: ros2_control-specific parameters

## Launch Files

The following is a list of the Python launch files and their purpose:

1. `display.launch.py`: Displays the robot in RVIZ2.  Also spawns the `Joint
   State Publisher` GUI to allow a user to control each of the wheels.

2. `gazebo.launch.py`: Starts the Gazebo simulation and spawns the robot.

## Changelog
`0.6.0` - `robomagellan.urdf.xacro` has corrected wheel inertial matrices and
wheel collision layers are now spherical rather than cylindrical.

`0.5.0` - Changing Gazebo's physics engine from Dart -> Bullet for mimic joints
to work.

`0.4.0` - Adding `sim.rviz`.

`0.3.0` - Adding `ros2_control` and `gazebo` plugins to URDFs.

`0.2.0` - Updating `robomagellan.urdf.xacro` to remove the `chassis` link,
correct the `base_link` -> `base_footprint` transform, and fixed the wheels'
frame orientations.  Added `display.launch.py` and `gazebo.launch.py` to work
with ROS 2 Jazzy.

`0.1.0` - Initial package and description
