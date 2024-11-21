cmake_minimum_required(VERSION 3.8)
project(cabot_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)
add_executable(cabot_can_node src/cabot_can_node.cpp)
ament_target_dependencies(cabot_can_node rclcpp std_msgs sensor_msgs std_srvs diagnostic_updater)

install(TARGETS cabot_can_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()