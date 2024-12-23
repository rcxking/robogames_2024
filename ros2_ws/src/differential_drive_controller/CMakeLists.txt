cmake_minimum_required(VERSION 3.8)
project(differential_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate ROS params from YAML config
generate_parameter_library(differential_drive_controller_parameters
  src/differential_drive_controller_parameter.yaml
)

# Differential Drive Controller plugin
add_library(differential_drive_controller SHARED
  src/differential_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(differential_drive_controller PUBLIC cxx_std_17)
target_include_directories(differential_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/differential_drive_controller>
)
target_link_libraries(differential_drive_controller PUBLIC
  differential_drive_controller_parameters
)
ament_target_dependencies(differential_drive_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(controller_interface
  differential_drive_plugin.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install include directory
install(
  DIRECTORY
    include
  DESTINATION
    include/differential_drive_controller
)

# Install differential drive controller plugin
install(
  TARGETS
    differential_drive_controller
    differential_drive_controller_parameters
  EXPORT
    export_differential_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_differential_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
