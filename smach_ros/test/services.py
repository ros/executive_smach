#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_ros')
import rospy
import rostest

import unittest

import std_srvs.srv as std_srvs

from smach import *
from smach_ros import *

from smach_msgs.msg import *

def empty_server(req):
    rospy.loginfo("Service called!")
    return std_srvs.EmptyResponse()

### Test harness
class TestServices(unittest.TestCase):
    def test_service_cb(self):
        """Test calling a service with a callback."""

        srv = rospy.Service('/empty', std_srvs.Empty, empty_server)

        sm = StateMachine(['succeeded','aborted','preempted','done'])
        with sm:
            def foo_response_cb(userdata, response):
                userdata.foo_var_out = 'foo!'
                return 'succeeded'

            StateMachine.add('FOO',
                    ServiceState('/empty',
                        std_srvs.Empty,
                        response_cb=foo_response_cb,
                        output_keys=['foo_var_out']),
                    remapping={'foo_var_out':'sm_var'},
                    transitions={'succeeded':'done'})

        outcome = sm.execute()

        rospy.logwarn("OUTCOME: "+outcome)

        assert outcome == 'done'

def main():
    rospy.init_node('services_test',log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'services_test', TestServices)

if __name__=="__main__":
    main();
