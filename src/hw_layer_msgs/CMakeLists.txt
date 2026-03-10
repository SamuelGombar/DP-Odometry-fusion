cmake_minimum_required(VERSION 3.8)
project(hw_layer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComponentHeaderMsg.msg"
  "msg/ComponentStateMsg.msg"
  "msg/IMUMsg.msg"
  "msg/AddonRotationMsg.msg"
  "msg/AddonStateMsg.msg"
  "msg/FlipperMsg.msg"
  "msg/AllFlippersMsg.msg"
  "msg/ShiftMsg.msg"
  "msg/AllShiftsMsg.msg"
  "msg/TofMsg.msg"
  "msg/AllTofsMsg.msg"
  "msg/BatteryMsg.msg"
  "msg/CommandMsg.msg"
  "msg/LiftAddonLiftAxisBriefInfoMsg.msg"
  "msg/LiftAddonDrawerBriefInfoMsg.msg"
  "msg/LiftAddonInfoMsg.msg"
  "msg/LiftMsg.msg"
  "msg/MotorsVelocityMsg.msg"
  "msg/VelocitiesComparisonMsg.msg"
  "msg/WheelsMsg.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
