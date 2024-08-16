cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_interface)

add_compile_options(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  control_msgs
  control_toolbox
  controller_manager
  hardware_interface
  roscpp
  sensor_msgs
  std_msgs
  transmission_interface
  urdf
)

###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    control_msgs
    control_toolbox
    controller_manager
    hardware_interface
    roscpp
    sensor_msgs
    std_msgs
    transmission_interface
    urdf
  LIBRARIES
    generic_hw_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Generic Hardware Interface
add_library(generic_hw_interface
  src/generic_hw_interface.cpp
)
target_link_libraries(generic_hw_interface
  ${catkin_LIBRARIES}
)

# Diff drive hardware interface
#add_library(diff_drive_hw_interface
#  src/diff_drive_hw_interface.cpp
#)
#target_link_libraries(diff_drive_hw_interface
#  generic_hw_interface
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# Install libraries
install(
  TARGETS
    generic_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
