cmake_minimum_required(VERSION 3.5)

project(rtmonitor_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(rtmonitor_msgs
  "msg/LoopTime.msg"
  "msg/Elapsed.msg"
  "msg/RtmVector.msg"
  "srv/ReqLoopTime.srv"
  "srv/ReqElapsed.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
