cmake_minimum_required(VERSION 3.5)
project(robomagellan_controller)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
