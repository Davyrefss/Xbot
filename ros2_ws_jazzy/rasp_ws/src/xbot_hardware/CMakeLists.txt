cmake_minimum_required(VERSION 3.8)
project(xbot_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(xbot_serial REQUIRED)
find_package(pluginlib REQUIRED)

add_library(xbot_system SHARED
  src/xbot_system.cpp
)

target_compile_features(xbot_system PUBLIC cxx_std_17)
target_include_directories(xbot_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xbot_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  xbot_serial
)
target_compile_options(xbot_system PRIVATE -Wall -Wextra -Wpedantic)

pluginlib_export_plugin_description_file(hardware_interface xbot_system_plugin.xml)

install(TARGETS xbot_system
  EXPORT export_xbot_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_xbot_system HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(xbot_system)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle xbot_serial)

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

ament_package()
