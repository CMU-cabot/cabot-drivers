cmake_minimum_required(VERSION 3.8)
project(cabot_base)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

# Install Python executables
install(PROGRAMS
  scripts/cabot_features.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

ament_package()
