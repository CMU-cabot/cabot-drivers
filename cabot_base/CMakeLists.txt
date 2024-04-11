cmake_minimum_required(VERSION 3.8)
project(cabot_base)

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(cabot_handle_v3_lib SHARED
  src/cabot/cabot_handle_v3_node.cpp
  src/cabot/handle_v3.cpp
  src/cabot/event.cpp
)

ament_target_dependencies(cabot_handle_v3_lib
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/cabot_handle_v3_node.py
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_node(cabot_handle_v3_lib
  PLUGIN "CaBotHandleV3Node"
  EXECUTABLE cabot_handle_v3_node
)

install(TARGETS
  cabot_handle_v3_lib
  DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

ament_package()
