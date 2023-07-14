cmake_minimum_required(VERSION 3.22)

project(smach_msgs)

set(AMENT_DEPS
  ament_cmake)
foreach(dep IN ITEMS ${AMENT_DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SmachContainerInitialStatusCmd.msg"
  "msg/SmachContainerStructure.msg"
  "msg/SmachContainerStatus.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
