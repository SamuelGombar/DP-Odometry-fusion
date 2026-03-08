cmake_minimum_required(VERSION 3.5)
project(robot_control_cpp)

# Export compile commands so VS Code / clangd / IntelliSense can read include paths
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(amrapi_msgs REQUIRED)

add_executable(scan_to_pc src/scan_to_pc.cpp)
ament_target_dependencies(scan_to_pc rclcpp sensor_msgs laser_geometry tf2_ros)

# Include directories
include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS})

add_executable(teleop src/kobuki/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)

add_executable(pid_control src/kobuki/pid_control.cpp)
ament_target_dependencies(pid_control
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)
target_include_directories(pid_control PRIVATE include)

add_executable(odom_republisher src/bp/odom_republisher.cpp)
ament_target_dependencies(odom_republisher rclcpp nav_msgs tf2_geometry_msgs geometry_msgs tf2_ros)

add_executable(scan_republisher src/bp/scan_republisher.cpp)
ament_target_dependencies(scan_republisher rclcpp sensor_msgs tf2_ros geometry_msgs)

add_executable(wheel_odom_publisher src/bp/wheel_odom_publisher.cpp)
ament_target_dependencies(wheel_odom_publisher
  rclcpp
  nav_msgs
  amrapi_msgs
  tf2
  tf2_ros
  geometry_msgs)

# publish a Path constructed from filtered odometry messages
add_executable(ekf_path src/ekf_path.cpp)
ament_target_dependencies(ekf_path
  rclcpp
  nav_msgs)

# publish a Path from raw wheel odometry
add_executable(wheel_odom_path src/bp/wheel_odom_path.cpp)
ament_target_dependencies(wheel_odom_path
  rclcpp
  nav_msgs)

# publish static TF tree: map -> odom -> base_link -> base_laser
add_executable(tree_publisher src/bp/tree_publisher.cpp)
ament_target_dependencies(tree_publisher
  rclcpp
  tf2_ros
  geometry_msgs)

install(TARGETS 
  teleop
  pid_control
  scan_to_pc
  odom_republisher
  scan_republisher
  wheel_odom_publisher
  ekf_path
  wheel_odom_path
  tree_publisher
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
