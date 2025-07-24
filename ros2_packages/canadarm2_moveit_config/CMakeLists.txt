cmake_minimum_required(VERSION 3.22)
project(canadarm2_moveit_config)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/moveit_topic_monitor.py
  scripts/moveit_isaac_bridge.py
  scripts/test_trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
