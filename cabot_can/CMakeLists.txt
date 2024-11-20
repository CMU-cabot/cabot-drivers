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

add_executable(can_all_node src/can_all_node.cpp)
ament_target_dependencies(can_all_node rclcpp std_msgs sensor_msgs std_srvs)

install(TARGETS can_all_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
