cmake_minimum_required(VERSION 3.8)
project(xbot_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
