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

### Custom tate classe
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
        State.__init__(self,['done','preempted'],['a'],['b'])
    def execute(self,ud):
        while 'a' not in ud:
            rospy.loginfo("Waiting for key 'a' to appear. ")
            rospy.sleep(0.1)
        ud.b = ud.a
        rospy.sleep(1.0)
        if self.preempt_requested():
            return 'preempted'
        return 'done'

### Test harness
class TestStateMachine(unittest.TestCase):
    def test_concurrence(self):
        """Test concurrent container."""
        sm = StateMachine(['done','succeeded'])
        with sm:
            cc = Concurrence(['succeeded','done'],
                    default_outcome = 'done',
                    outcome_map = {'succeeded':{'SETTER':'done'}})
            sm.add('CONCURRENT',cc)
            with cc:
                Concurrence.add('SETTER', Setter())
                Concurrence.add('GETTER', Getter())

        outcome = sm.execute()

        assert outcome == 'succeeded'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

    def test_preempt(self):
        """Test concurrent container that preempts siblings."""
        cc = Concurrence(['succeeded','done'],
                default_outcome = 'done',
                child_termination_cb = lambda so: True,
                outcome_map = {'succeeded':{'SETTER':'done', 'GETTER':'preempted'}})
        with cc:
            Concurrence.add('SETTER', Setter())
            Concurrence.add('GETTER', Getter())

        outcome = cc.execute()

        assert outcome == 'succeeded'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

    def test_no_preempt(self):
        """Test concurrent container that doesnt preempt siblings."""
        cc = Concurrence(['succeeded','done'],
                default_outcome = 'done',
                child_termination_cb = lambda so: False,
                outcome_map = {
                    'succeeded':{
                        'SETTER':'done',
                        'GETTER':'done'}})
        with cc:
            Concurrence.add('SETTER', Setter())
            Concurrence.add('GETTER', Getter())

        outcome = cc.execute()

        assert outcome == 'succeeded'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

    def test_outcome_cb(self):
        """Test concurrent container that doesnt preempt siblings."""
        cc = Concurrence(['succeeded','done'],
                default_outcome = 'done',
                child_termination_cb = lambda so: False,
                outcome_cb = lambda so: list(set(so.values()))[0])
        with cc:
            Concurrence.add('SETTER', Setter())
            Concurrence.add('GETTER', Getter())

        outcome = cc.execute()

        assert outcome == 'done'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

def main():
    rospy.init_node('concurrence_test',log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'concurrence_test', TestStateMachine)

if __name__=="__main__":
    main();
