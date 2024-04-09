cmake_minimum_required(VERSION 3.8)
project(cabot_base)

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(cabot_handle_v2_lib SHARED
  src/cabot/cabot_handle_v2_node.cpp
  src/cabot/handle_v2.cpp
  src/cabot/event.cpp
)

ament_target_dependencies(cabot_handle_v2_lib
  rclcpp
  rclcpp_components
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/cabot_handle_v2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_node(cabot_handle_v2_lib
  PLUGIN "CaBotHandleV2Node"
  EXECUTABLE cabot_handle_v2_node
)

install(TARGETS
  cabot_handle_v2_lib
  DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

ament_package()
