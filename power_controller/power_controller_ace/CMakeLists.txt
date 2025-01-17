cmake_minimum_required(VERSION 3.8)
project(power_controller_ace)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/power_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
