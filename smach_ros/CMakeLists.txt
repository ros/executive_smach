cmake_minimum_required(VERSION 2.8.3)

project(smach_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy rostopic std_msgs std_srvs actionlib actionlib_msgs smach smach_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/state_machine.test)
  add_rostest(test/sequence.test)
  add_rostest(test/concurrence.test)
  add_rostest(test/introspection.test)
  add_rostest(test/smach_actionlib.test)
  add_rostest(test/monitor.test)
endif()
