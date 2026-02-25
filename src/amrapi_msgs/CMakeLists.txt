cmake_minimum_required(VERSION 3.8)
project(amrapi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(alert_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ApiHistoryMsg.msg"
  "msg/AtSegmentMsg.msg"
  "msg/AtStationMsg.msg"
  "msg/AtStationTelemetryMsg.msg"
  "msg/BatteryMsg.msg"
  "msg/BlockedMsg.msg"
  "msg/DetectedAmrNames.msg"
  "msg/ImageFrameMsg.msg"
  "msg/LiftAddonDrawerMsg.msg"
  "msg/MicroAdjustmentMsg.msg"
  "msg/MotorMsg.msg"
  "msg/MotorsMsg.msg"
  "msg/PathMsg.msg"
  "msg/PathSyncPointMsg.msg"
  "msg/RestrictionSetMsg.msg"
  "msg/ResultMsg.msg"
  "msg/RobotAlertSummaryMsg.msg"
  "msg/RobotModelMsg.msg"
  "msg/RobotStateMsg.msg"
  "msg/SafetyMsg.msg"
  "msg/SecondaryMotorBlockingSourcesMsg.msg"
  "msg/StateMsg.msg"
  "msg/TachometerMsg.msg"
  "msg/TemporaryWaypointMsg.msg"
  "msg/TrajectoryMsg.msg"
  "msg/VehicleVelocityMsg.msg"
  "msg/Velocity.msg"
  "msg/WaypointMsg.msg"
  "msg/ZoneMsg.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs alert_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
