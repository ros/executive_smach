
project(smach_msgs)

if(USE_ROSBUILD)
  cmake_minimum_required(VERSION 2.4.6)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_genmsg()
else()
  cmake_minimum_required(VERSION 2.8.3)
  find_package(catkin REQUIRED COMPONENTS 
    message_generation std_msgs)

  add_message_files(FILES
    SmachContainerInitialStatusCmd.msg
    SmachContainerStructure.msg
    SmachContainerStatus.msg)

  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(CATKIN_DEPENDS message_runtime)

endif()
