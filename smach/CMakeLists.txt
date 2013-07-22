
project(smach)

# Check if we should build with rosbuild or with catkin
if(USE_ROSBUILD)
  cmake_minimum_required(VERSION 2.4.6)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

else()

  cmake_minimum_required(VERSION 2.8.3)
  find_package(catkin REQUIRED COMPONENTS)

  catkin_python_setup()

  catkin_package()
  
endif()
