cmake_minimum_required(VERSION 3.0.2)
project(differential_drive_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  tf
  urdf
)

find_package(Boost REQUIRED)

# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(cfg/DifferentialDriveController.cfg)

# Catkin package declaration
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    realtime_tools
    tf
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
)

###########
## Build ##
###########

# Header include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Differential Drive controller plugin
add_library(${PROJECT_NAME}
  src/DifferentialDriveController.cpp
  src/Odometry.cpp
  src/SpeedLimiter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

#############
## Testing ##
#############

# TODO: Add tests here

#############
## Install ##
#############

# Install headers
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install libraries
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(
  FILES
    differential_drive_controller_plugins.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
