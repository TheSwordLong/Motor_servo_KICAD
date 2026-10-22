cmake_minimum_required(VERSION 3.10...3.22)
project(robot_motor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChannelConfig.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  "msg/SelftestChannel.msg"
  "srv/Stop.srv"
  "srv/ClearFault.srv"
  "srv/SetZero.srv"
  "srv/RetrySave.srv"
  "srv/SetBrake.srv"
  "srv/SetDirection.srv"
  "srv/SetManualAdjust.srv"
  "msg/ManualAdjustState.msg"
  "srv/ApplyConfig.srv"
  "action/MoveJoint.action"
  "action/MoveJointDeg.action"
  "msg/MotorStateDeg.msg"
  "msg/MotorStatesDeg.msg"
  "action/RunSelfTest.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
