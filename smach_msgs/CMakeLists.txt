cmake_minimum_required(VERSION 3.5)

project(smach_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SmachContainerInitialStatusCmd.msg"
  "msg/SmachContainerStructure.msg"
  "msg/SmachContainerStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
