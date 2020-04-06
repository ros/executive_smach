#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_ros')
import rospy
import rostest

import threading

import unittest

from smach import *
from smach_ros import *

from smach_msgs.msg import *

### Custom state classe
class Setter(State):
    """State that sets the key 'a' in its userdata"""
    def __init__(self):
        State.__init__(self,['done'],[],['a'])
    def execute(self,ud):
        ud.a = 'A'
        rospy.loginfo("Added key 'a'.")
        return 'done'

class Getter(State):
    """State that grabs the key 'a' from userdata, and sets 'b'"""
    def __init__(self):
        State.__init__(self,['done'],['a'],['b'])
    def execute(self,ud):
        while 'a' not in ud and not rospy.is_shutdown():
            #rospy.loginfo("Waiting for key 'a' to appear. ")
            rospy.sleep(0.1)
        ud.b = ud.a
        return 'done'

### Test harness
class TestIntrospection(unittest.TestCase):

    def test_introspection(self):
        """Test introspection system."""
        # Construct state machine
        sm = StateMachine(['done'])
        sm2 = StateMachine(['done'])
        sm3 = StateMachine(['done'])

        with sm:
            # Note: the following "Getter" state should fail
            StateMachine.add('GETTER1', Getter(), {})
            StateMachine.add('SM2', sm2, {'done':'SM3'})
            with sm2:
                StateMachine.add("SETTER", Setter(), {})
            StateMachine.add('SM3', sm3, {'done':'done'})
            with sm3:
                StateMachine.add("SETTER", Setter(), {})
            StateMachine.add('GETTER2', Getter(), {'done':'SM2'})

        sm.set_initial_state(['GETTER1'])
        sm2.set_initial_state(['SETTER'])
        sm3.set_initial_state(['SETTER'])

        # Run introspector
        intro_server = IntrospectionServer('intro_test',sm,'/intro_test')
        server_thread = threading.Thread(target=intro_server.start)
        server_thread.start()

        intro_client = IntrospectionClient()
        servers = intro_client.get_servers()
        while '/intro_test' not in servers and not rospy.is_shutdown():
            servers = intro_client.get_servers()
            rospy.loginfo("Smach servers: "+str())
            rospy.sleep(0.1)

        assert '/intro_test' in servers

        # Set initial state
        injected_ud = UserData()
        injected_ud.a = 'A'
        init_set = intro_client.set_initial_state('intro_test','/intro_test',['SM2'],injected_ud,timeout = rospy.Duration(10.0))
        assert init_set

        outcome = sm.execute()

        assert outcome == 'done'


def main():
    rospy.init_node('introspection_test',log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'introspection_test', TestIntrospection)

if __name__=="__main__":
    main();
