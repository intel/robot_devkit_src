cmake_minimum_required(VERSION 3.5)
project(rtmonitor_test)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtmonitor REQUIRED)
find_package(rtmonitor_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

set(dependencies rclcpp
  rclcpp
  rclcpp_lifecycle
  std_msgs
  rtmonitor
  rtmonitor_msgs
)

add_executable(test_rtmonitor
  src/test_rtm_core.cpp)

ament_target_dependencies(test_rtmonitor
  ${dependencies}
)

add_executable(test_looptime
  src/test_looptime.cpp)

ament_target_dependencies(test_looptime
  ${dependencies}
)

add_executable(test_latency
  src/test_latency.cpp)

ament_target_dependencies(test_latency
  ${dependencies}
)

add_executable(test_elapsed
  src/test_elapsed.cpp)

ament_target_dependencies(test_elapsed
  ${dependencies}
)

#[[
add_executable(test_client_server
  src/test_client_server.cpp)

ament_target_dependencies(test_client_server
  "rclcpp"
  "rclcpp_lifecycle"
  "rtmonitor"
  "rtmonitor_msgs"
  "std_msgs")

add_executable(test_client_server2
  src/test_client_server2.cpp)

ament_target_dependencies(test_client_server2
  "rclcpp"
  "rclcpp_lifecycle"
  "rtmonitor"
  "rtmonitor_msgs"
  "std_msgs")

add_executable(test_elapsed_global
  src/test_elapsed_global.cpp)

ament_target_dependencies(test_elapsed_global
  "rclcpp"
  "rclcpp_lifecycle"
  "rtmonitor"
  "rtmonitor_msgs"
  "std_msgs")

add_executable(test_ros2_comm
  src/test_ros2_comm/main.cpp)

ament_target_dependencies(test_ros2_comm
  "rclcpp"
  "rclcpp_lifecycle"
  "rtmonitor"
  "rtmonitor_msgs"
  "std_msgs")
]]
install(TARGETS
  test_rtmonitor
  test_looptime
  test_latency
  test_elapsed
#  test_client_server
#  test_client_server2
#  test_elapsed_global
#  test_ros2_comm
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
