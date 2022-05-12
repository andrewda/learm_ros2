cmake_minimum_required(VERSION 3.5)
project(learm_ros2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
