# robomagellan_firmware

This package contains the code that implements the real robot's ROS 2 hardware
interfaces.

## Hardware Interfaces

The main hardware interfaces are implemented by `robomagellan_interface.hpp` and
`robomagellan_interface.cpp`.  Since these are `pluginlib` plugins you also need
the `robomagellan_interface.xml`.

Because the robot has 2 motors there are 2 state interfaces (each motor has a
position/velocity interface) and 2 command interfaces (each motor can be sent a
desired wheel velocity in radians/second).  The differential drive controller
will provide the desired wheel velocities to these hardware interfaces.

## Arduino connection

An Arduino Mega connects to the motor controllers and the quadrature encoders.
This Arduino is connected to the Raspberry Pi 5 via a USB cable.

## Launch Files

`hardware_interface.launch.py` launches the `ros2_control` `ControllerManager`.
The hardware interface plugin is loaded from the robot's URDF
(`robomagellan.urdf.xacro` within the `robomagellan_description` package).

## Changelog
`0.4.0` - Implemented `RobomagellanInterface::write()` to send desired motor
elocities to the Arduino.

`0.3.0` - Implemented `RobomagellanInterface::read()` to get motor velocities
and compute/update wheel velocity/position state interfaces.

`0.2.0` - `libserial` connections/close connections to the Arduino Mega.

`0.1.0` - Initial package
