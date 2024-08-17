cmake_minimum_required(VERSION 3.5)
project(auxiliary)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(waypoint_recorder_sim src/WaypointRecorder.cpp src/WaypointRecorder_main.cpp)
ament_target_dependencies(waypoint_recorder_sim 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros visualization_msgs
)

add_executable(waypoints_displayer_sim src/WaypointDisplayer.cpp src/FileHandler.cpp src/WaypointDisplayer_main.cpp)
ament_target_dependencies(waypoints_displayer_sim 
  rclcpp std_msgs visualization_msgs
)

# Install Cpp executables
install(TARGETS
  waypoint_recorder_sim
  waypoints_displayer_sim
  DESTINATION lib/${PROJECT_NAME})

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



ament_package()
