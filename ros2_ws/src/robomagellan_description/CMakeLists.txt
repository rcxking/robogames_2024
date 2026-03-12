cmake_minimum_required(VERSION 3.8)
project(robomagellan_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# Install directories
install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
