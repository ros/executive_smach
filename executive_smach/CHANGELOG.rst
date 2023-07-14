^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package executive_smach
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.3 (2023-07-14)
------------------
* Fix: Buildfarm issue https://github.com/ros/executive_smach/issues/111

3.0.2 (2023-06-29)
------------------
* Fix `#92 <https://github.com/ros/executive_smach/issues/92>`_ 

3.0.1 (2023-06-12)
------------------
* Add license file.

3.0.0 (2023-06-12)
------------------
* ROS2-ified. `#101 <https://github.com/ros/executive_smach/issues/101>`_

2.5.1 (2023-02-15)
------------------

2.5.0 (2020-05-14)
------------------
* Python 3 compatibility `#71 <https://github.com/ros/executive_smach/issues/71>`_
* Bump CMake version to avoid CMP0048 warning
* Contributors: Shane Loretz, ahcorde

2.0.1 (2017-06-08)
------------------
* [fix] SimpleActionState will wait forever for a missing ActionServer `#41 <https://github.com/ros/executive_smach/pull/41>`_
* [fix] monitor state callback args and adding unit test for monitor state that modifies userdata
* [improve] Specify queue sizes for introspection publishers `#31 <https://github.com/ros/executive_smach/pull/31>`_
* [improve] make ServiceState more robust to termination while waiting for service `#32 <https://github.com/ros/executive_smach/pull/32>`_
* [build] make rostest in CMakeLists optional `#45 <https://github.com/ros/executive_smach/pull/45>`_
* change MonitorState and document its behavior 
* increment n_checks only *after* checking
  otherwise, setting max_checks=1 results in a MonitorState that returns the 'valid' outcome for any message
* [test] adding test for actionlib timeout
* [maintenance] Update maintainer. switching to package.xml format 2
* Contributors: Isaac I.Y. Saito, Jonathan Bohren, Loy, Lukas Bulwahn, Nils Berg, contradict

2.0.0 (2014-04-17)
------------------

1.3.1 (2013-07-22)
------------------
* adding changelogs
