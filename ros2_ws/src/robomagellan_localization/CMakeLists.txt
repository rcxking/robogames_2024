cmake_minimum_required(VERSION 3.8)
project(robomagellan_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# Odometry motion model Node
add_executable(odometry_motion_model
  src/odometry_motion_model.cpp
)
ament_target_dependencies(odometry_motion_model
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
)

# Install include directories
install(
  DIRECTORY
    include
  DESTINATION
    include
)

# Install executables
install(
  TARGETS
    odometry_motion_model
  DESTINATION
    lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
