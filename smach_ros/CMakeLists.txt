cmake_minimum_required(VERSION 2.8.3)

project(smach_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy rostopic std_msgs std_srvs actionlib actionlib_msgs smach smach_msgs
)
