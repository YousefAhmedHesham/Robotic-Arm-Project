cmake_minimum_required(VERSION 3.22)
project(robot_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

# Install launch directory correctly
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install other necessary files
install(PROGRAMS 
  scripts/robot_moveit_python.py
  scripts/robot_gui_control.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config scripts DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
