cmake_minimum_required(VERSION 3.5)

project(rtmonitor)

if(WIN32 OR APPLE OR ANDROID)
  message(STATUS "rtmonitor is only supported on Linux, skipping...")
  return()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake QUIET)
find_package(rclcpp REQUIRED)
find_package(rtmonitor_msgs REQUIRED)

include_directories(include)
add_library(rtmonitor SHARED src/rtmonitor.cpp)
ament_target_dependencies(rtmonitor
  rclcpp
  rtmonitor_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rtmonitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
set(exported_libraries rtmonitor pthread)
ament_export_libraries(${exported_libraries})
ament_package()


