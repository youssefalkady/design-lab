cmake_minimum_required(VERSION 3.10)
project(turtle_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetPose.msg"
  "action/GoToPose.action"
  "action/DrawSquare.action"
  "srv/SetSpeedLimits.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
