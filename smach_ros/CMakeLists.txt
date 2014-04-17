cmake_minimum_required(VERSION 2.8.3)

project(smach_ros)

find_package(catkin REQUIRED COMPONENTS rostest)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy rostopic std_msgs std_srvs actionlib actionlib_msgs smach smach_msgs
)

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/state_machine.test)
  add_rostest(test/sequence.test)
  add_rostest(test/concurrence.test)
  add_rostest(test/introspection.test)
  add_rostest(test/smach_actionlib.test)
endif()
