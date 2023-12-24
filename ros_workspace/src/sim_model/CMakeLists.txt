cmake_minimum_required(VERSION 3.0.2)
project(sim_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_model
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Install Gazebo Models
add_custom_target(ALL)
add_custom_command(TARGET ALL POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E copy
#          robomagellan_model/*
#          /home/bryant/.gazebo/models/robomagellan_model
COMMAND ${CMAKE_COMMAND} -E echo "I'M EXECUTING"
)
