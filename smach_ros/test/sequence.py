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
        State.__init__(self,['done'],['a'],['b'])
    def execute(self,ud):
        while 'a' not in ud:
            #rospy.loginfo("Waiting for key 'a' to appear. ")
            rospy.sleep(0.1)
        ud.b = ud.a
        return 'done'

### Test harness
class TestSequence(unittest.TestCase):
    def test_sequence(self):
        """Test adding a sequence of states."""
        sq = Sequence(['succeeded','aborted','preempted','done'],connector_outcome='succeeded')
        with sq:
            Sequence.add('FIRST', SimpleActionState('reference_action',TestAction, goal = g1))
            Sequence.add('SECOND', SimpleActionState('reference_action',TestAction, goal = g1))
            Sequence.add('THIRD', SimpleActionState('reference_action',TestAction, goal = g1),{'succeeded':'done'})
        outcome = sq.execute()

        assert outcome == 'done'

    def test_exception(self):
        class DoneState(State):
            def __init__(self):
                State.__init__(self,outcomes=['done'])
            def execute(self,ud=None):
                return 'done'

        class ErrorState(State):
            """State falls with exception"""
            def __init__(self):
                State.__init__(self, ['done'])
            def execute(self, ud):
                raise Exception('Test exception')

        sq = Sequence(['done'], connector_outcome='done')
        with sq:
            Sequence.add('OK', DoneState())
            Sequence.add('ERROR', ErrorState())
            Sequence.add('IGNORED', Setter())

        with self.assertRaises(InvalidUserCodeError):
            sq.execute()

        assert sq.is_running() == False
        assert 'a' not in sq.userdata  # test IGNORED state wasn't called

def main():
    rospy.init_node('sequence_test',log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'sequence_test', TestSequence)

if __name__=="__main__":
    main();
