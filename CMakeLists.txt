# Prevent this CMakeLists 
if(USE_ROSBUILD)
  cmake_minimum_required(VERSION 2.4.6)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_make_distribution(1.3.0)
endif()
