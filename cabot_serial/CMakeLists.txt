cmake_minimum_required(VERSION 3.8)
project(cabot_serial)

set(cabot_serial_MAJOR_VERSION 1)
set(cabot_serial_MINOR_VERSION 0)
set(cabot_serial_PATCH_VERSION 0)
set(cabot_serial_VERSION
  ${cabot_serial_MAJOR_VERSION}.${cabot_serial_MINOR_VERSION}.${cabot_serial_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

set(dependencies
  diagnostic_updater
  sensor_msgs
  std_msgs
  std_srvs
  rclcpp
  rclcpp_components
)

include_directories(
  include
)

add_library(cabot_serial_lib SHARED
  src/cabot_serial.cpp
  src/arduino_serial.cpp
)

ament_target_dependencies(cabot_serial_lib
  ${dependencies}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/cabot_serial_node.py
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_node(cabot_serial_lib
  PLUGIN "CaBotSerialNode"
  EXECUTABLE cabot_serial_node
)

# Install executables
install(TARGETS
  cabot_serial_lib
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include/
)

ament_export_include_directories("include/")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
