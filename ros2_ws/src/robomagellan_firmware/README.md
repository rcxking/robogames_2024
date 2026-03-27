# robomagellan_firmware

This package contains the code that implements the real robot's ROS 2 hardware
interfaces.

## Hardware Interfaces

The main hardware interfaces are implemented by `robomagellan_interface.hpp` and
`robomagellan_interface.cpp`.  Since these are `pluginlib` plugins you also need
the `robomagellan_interface.xml`.

## Launch Files

`hardware_interface.launch.py` launches the `ros2_control` `ControllerManager`.
The hardware interface plugin is loaded from the robot's URDF
(`robomagellan.urdf.xacro` within the `robomagellan_description` package).

## Changelog
`0.2.0` - `libserial` connections/close connections to the Arduino Mega.

`0.1.0` - Initial package
