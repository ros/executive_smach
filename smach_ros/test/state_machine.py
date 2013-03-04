#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_ros')
import rospy
import rostest

import unittest

from actionlib import *
from actionlib.msg import *

from smach import *
from smach_ros import *

from smach_msgs.msg import *

# Static goals
g1 = TestGoal(1) # This goal should succeed
g2 = TestGoal(2) # This goal should abort
g3 = TestGoal(3) # This goal should be rejected

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
        while 'a' not in ud:
            #rospy.loginfo("Waiting for key 'a' to appear. ")
            rospy.sleep(0.1)
        ud.b = ud.a
        return 'done'

### Test harness
class TestStateMachine(unittest.TestCase):
    def test_userdata(self):
        """Test serial manipulation of userdata."""
        sm = StateMachine(['done'])
        with sm:
            StateMachine.add('SETTER', Setter(),{'done':'GETTER'})
            StateMachine.add('GETTER', Getter(),{})

        outcome = sm.execute()

        assert outcome == 'done'
        assert 'a' in sm.userdata
        assert sm.userdata.a == 'A'
        assert sm.userdata.b == 'A'

    def test_userdata_nesting(self):
        """Test serial manipulation of userdata."""
        sm = StateMachine(['done','preempted','aborted'])
        with sm:
            StateMachine.add('SETTER', Setter(),{'done':'GETTER'})
            StateMachine.add('GETTER', Getter(),{'done':'NEST'})

            sm2 = StateMachine(['done','aborted','preempted'])
            sm2.register_input_keys(['a'])
            with sm2:
                StateMachine.add('ASSERTER', ConditionState(
                    lambda ud: 'a' in ud,
                    input_keys = ['a']),
                    {'true':'done','false':'aborted'})
            StateMachine.add('NEST',sm2)

        outcome = sm.execute()

        assert outcome == 'done'
        assert 'a' in sm.userdata
        assert 'b' in sm.userdata
        assert sm.userdata.a == 'A'
        assert sm.userdata.b == 'A'

    def test_userdata_nesting2(self):
        """Test setting of userdata manually on construction."""
        sm = StateMachine(['done','aborted','preempted'])
        sm.userdata.foo = 1
        with sm:
            StateMachine.add('SETTER', Setter(),{'done':'GETTER'})
            StateMachine.add('GETTER', Getter(),{'done':'NEST'})
            
            sm2 = StateMachine(['done','aborted','preempted'])
            sm2.register_input_keys(['foo'])
            with sm2:
                StateMachine.add('ASSERTER', ConditionState(
                    lambda ud: 'foo' in ud,
                    input_keys = ['foo']),
                    {'true':'done','false':'aborted'})
            StateMachine.add('NEST',sm2)

        outcome = sm.execute()

        assert outcome == 'done'
        assert 'foo' in sm.userdata

    def test_userdata_remapping(self):
        """Test remapping of userdata."""
        sm = StateMachine(['done','preempted','aborted'])
        with sm:
            StateMachine.add('SETTER', Setter(), {'done':'GETTER'}, remapping = {'a':'x'})
            StateMachine.add('GETTER', Getter(), {'done':'done'}, remapping = {'a':'x','b':'y'})

        outcome = sm.execute()

        assert outcome == 'done'
        assert 'x' in sm.userdata
        assert 'y' in sm.userdata
        assert sm.userdata.x == 'A'
        assert sm.userdata.y == 'A'

    def test_sequence(self):
        """Test adding a sequence of states."""
        sm = StateMachine(['succeeded','aborted','preempted','done'])
        with sm:
            StateMachine.add_auto('FIRST', SimpleActionState('reference_action',TestAction, goal = g1),['succeeded'])
            StateMachine.add_auto('SECOND', SimpleActionState('reference_action',TestAction, goal = g1),['succeeded'])
            StateMachine.add('THIRD', SimpleActionState('reference_action',TestAction, goal = g1),{'succeeded':'done'})
        outcome = sm.execute()

        assert outcome == 'done'

    def test_group(self):
        """Test adding a bunch of states with group args."""

        class DoneState(State):
            def __init__(self):
                State.__init__(self,outcomes=['done'])
            def execute(self,ud=None):
                return 'done'

        sm = StateMachine(['succeeded','done'])
        with sm:
            StateMachine.add('FAILSAUCE',DoneState())
            transitions = {'aborted':'FAILSAUCE','preempted':'FAILSAUCE'}
            with sm:
                StateMachine.add('FIRST', SimpleActionState('reference_action',TestAction, goal = g1), transitions)
                StateMachine.add('SECOND', SimpleActionState('reference_action',TestAction, goal = g2), transitions)
                StateMachine.add('THIRD', SimpleActionState('reference_action',TestAction, goal = g1), transitions)
        outcome = sm.execute()

        assert outcome == 'done'

    def test_alt_api(self):
        """Test adding with alt apis."""

        sm = StateMachine(['succeeded','aborted','preempted'])
        with sm.opened():
            sm.add('FIRST', SimpleActionState('reference_action',TestAction, goal = g1),{})
            sm.add('SECOND', SimpleActionState('reference_action',TestAction, goal = g2),{})
            sm.add('THIRD', SimpleActionState('reference_action',TestAction, goal = g1),{})
        outcome = sm.execute()

        assert outcome == 'succeeded'

def main():
    rospy.init_node('state_machine_test',log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'state_machine_test', TestStateMachine)

if __name__=="__main__":
    main();
