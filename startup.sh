#!/usr/bin/bash

# startup.sh - Launches the ROS code for the RoboMagellan competition and any
# other needed daemons.
#
# Bryant Pong
# 2/1/24

# Start the PI GPIO daemon if it hasn't already been started
echo "Checking if PI GPIO daemon is running"
if pgrep -x "pigpiod" > /dev/null
then
  echo "PI GPIO daemon is already running"
else
  echo "PIO GPIO daemon not running.  Starting"
  sudo pigpiod
fi

# Launch the ROS nodes
roslaunch competition_files robomagellan.launch enable_gps:=false
