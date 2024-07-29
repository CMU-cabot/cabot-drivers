cmake_minimum_required(VERSION 3.8)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(NODES set_vel_gains_node)
foreach(target IN LISTS NODES)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
