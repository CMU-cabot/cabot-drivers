cmake_minimum_required(VERSION 3.5)
project(wireless_scanner_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
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

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  script/receiver/ble_scan_converter.py
  script/receiver/esp32_wifi_scan_converter.py
  script/scanner/dbus_ibeacon_scanner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
