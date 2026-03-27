# robomagellan_bringup

This package contains the launch files to start either the real robot or a
simulated robot.

## Launch Files

The following is a list of the Python launch files and their purpose:

1. `competition.launch.py` - Real robot startup
2. `sim.launch.py` - Gazebo/simulated robot startup

## Changelog
`0.5.0` - `competition.launch.py` now spawns the hardware interface and joystick
nodes

`0.4.0` - `sim.launch.py` spawns Rviz2 node

`0.3.0` - Spawning `joy`/`joy_teleop` nodes for Xbox controller simulation
control

`0.2.0` - Initial `sim.launch.py`

`0.1.0` - Initial package and `competition.launch.py`
