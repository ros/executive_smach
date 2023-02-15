^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smach
^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.0 (2020-05-14)
------------------
* Python 3 compatibility `#71 <https://github.com/ros/executive_smach/issues/71>`_
* Use setuptools instead of distutils
* Bump CMake version to avoid CMP0048 warning
* Update state.py Docstrings' @type descriptions `#59 <https://github.com/ros/executive_smach/issues/59>`_
* Typo set_shutdown_cb() --> set_shutdown_check() `#56 <https://github.com/ros/executive_smach/issues/56>`_
* Contributors: Isaac I.Y. Saito, Joseph Coombe, Shane Loretz, ahcorde, cclauss

2.0.1 (2017-06-08)
------------------
* [maintenance] Update maintainer. switching to package.xml format 2
* Contributors: Isaac I.Y. Saito

2.0.0 (2014-04-17)
------------------
* Merging changes, resolving conflicts, from strands-project (@cburbridge)
* cleaning up and removing rosbuild support
* merging groovy and hydro
* Fix get_internal_edges returning list of tuples, not list of lists
* Remove old methods set_userdata
* Remove superfluous parent class declaration 'UserData' from 'Remapper'
* Add local error base class 'SmachError', extending Exception
* Fix syntax errors, doc typos and indentations glitches
* Fixed invalid exception type in concurrence.py
* Checking threads have fully terminated before cleanup of outcomes dict
  This commit uses thread.isAlive() on each concurrent state runner to check for termination of all the threads before continuing. This is necessary as only checking that the outcome has been filled in does not mean the thread has completed; if the thread has not completed it may not yet have called the termination callback. If this loop exits before the termination callback of the last thread is called, then the callback will occasionally be sent an empty dictionary (when the main thread has got to line 305).
* cope with missed state termination notifications
  Concurrent states could terminate and notify _ready_event without the concurrence container realising, as it could be busy checking the outcome values. This makes the concurrency container get stuck on line 250. This commit adds a timeout to the wait to safely cope with missing notifications.
* Adding event for thread synchronization in concurrence and using event not condition in monitor state
* Contributors: Felix Kolbe, Jonathan Bohren, Piotr Orzechowski, cburbridge

1.3.1 (2013-07-22)
------------------
* adding changelogs
* added missing catkin_package() calls in CMakeLists.txt files of packages smach and smach_ros
* Updating maintainer name

* added missing catkin_package() calls in CMakeLists.txt files of packages smach and smach_ros
* Updating maintainer name
