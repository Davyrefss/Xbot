cmake_minimum_required(VERSION 3.8)
project(xbot_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出 compile_commands.json, 供 VSCode 的 C/C++ IntelliSense 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

add_library(xbot_serial_core SHARED
  src/protocol.cpp
  src/serial_port.cpp
  src/serial_client.cpp  
)

target_compile_features(xbot_serial_core PUBLIC cxx_std_17)
target_include_directories(xbot_serial_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(xbot_serial_core PUBLIC Threads::Threads)
target_compile_options(xbot_serial_core PRIVATE -Wall -Wextra -Wpedantic)

# keep orignal interactive protocol debugger as a separate executable.
add_executable(manual_sender tools/manual_sender.cpp)
target_compile_features(manual_sender PRIVATE cxx_std_17)
target_link_libraries(manual_sender xbot_serial_core Threads::Threads)
ament_target_dependencies(manual_sender rclcpp)

install(TARGETS xbot_serial_core
  EXPORT export_xbot_serial_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS manual_sender DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_xbot_serial_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(xbot_serial_core)
ament_export_dependencies(Threads)

ament_package()
