cmake_minimum_required(VERSION 3.8)
project(arm_ur_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY srdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
