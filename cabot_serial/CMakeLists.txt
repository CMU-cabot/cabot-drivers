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

set(dependencies
  diagnostic_updater
  sensor_msgs
  std_msgs
  std_srvs
  rclcpp
)

include_directories(
  include
)

add_executable(cabot_serial_node
  src/cabot_serial.cpp
  src/arduino_serial.cpp
)

ament_target_dependencies(cabot_serial_node
  ${dependencies}
)

# Install executables
install(TARGETS
  cabot_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include/")
ament_export_libraries(cabot)
ament_export_dependencies(${dependencies})

ament_package()
