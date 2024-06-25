cmake_minimum_required(VERSION 3.8)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(set_vel_gains_node src/set_vel_gains_node.cpp)
ament_target_dependencies(set_vel_gains_node rclcpp std_msgs)
install(TARGETS
  set_vel_gains_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS set_vel_gains_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()