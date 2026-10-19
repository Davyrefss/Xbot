cmake_minimum_required(VERSION 3.8)
project(xbot_serial)

# 导出 compile_commands.json, 供 VSCode 的 C/C++ IntelliSense 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# serial_port 串口封装库
add_library(serial_port src/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# protocol 帧协议编解码库(与下位机 comm_protocol.c 逐字节等价)
add_library(protocol src/protocol.cpp)
target_include_directories(protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# serial_node 串口通信节点
add_executable(serial_node src/serial_node.cpp)
target_link_libraries(serial_node protocol serial_port)
ament_target_dependencies(serial_node rclcpp geometry_msgs nav_msgs tf2_ros)

# manual_sender 手动发帧调试节点(键盘输入文本命令)
add_executable(manual_sender src/manual_sender.cpp)
target_link_libraries(manual_sender protocol serial_port)
ament_target_dependencies(manual_sender rclcpp)

install(TARGETS serial_node manual_sender
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS serial_port protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)

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

  # protocol 模块自检(golden 帧向量 + 状态机边界用例)
  add_executable(protocol_test test/protocol_test.cpp)
  target_link_libraries(protocol_test protocol)
  add_test(NAME protocol_test COMMAND protocol_test)
endif()

ament_package()
