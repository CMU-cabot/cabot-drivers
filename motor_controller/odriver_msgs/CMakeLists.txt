cmake_minimum_required(VERSION 3.5)
project(odriver_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  ament_xmllint()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorTarget.msg"
  "msg/PIControlData.msg"
  DEPENDENCIES std_msgs
)

ament_package()
