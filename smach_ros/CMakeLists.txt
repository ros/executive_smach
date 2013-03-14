
project(smach_ros)

if(USE_ROSBUILD)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

else()

  cmake_minimum_required(VERSION 2.8.3)
  find_package(catkin REQUIRED COMPONENTS)

  catkin_python_setup()

endif()
