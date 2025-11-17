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

# Include directories
include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS})

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)

# Create library for kalman_filter
add_library(kalman_filter_lib src/kalman_filter.cpp)
target_link_libraries(kalman_filter_lib PUBLIC Eigen3::Eigen)
target_include_directories(kalman_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(fusion src/fusion.cpp)
ament_target_dependencies(fusion 
                          rclcpp 
                          geometry_msgs
                          nav_msgs 
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          sensor_msgs)
target_link_libraries(fusion kalman_filter_lib Eigen3::Eigen)
target_include_directories(fusion PRIVATE include)

add_executable(scan_printing src/scan_printing.cpp)
ament_target_dependencies(scan_printing rclcpp geometry_msgs sensor_msgs)

install(TARGETS 
    teleop
    fusion
    scan_printing
    kalman_filter_lib
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
