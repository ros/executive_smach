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

class AssertUDState(State):
    def __init__(self, keys):
        State.__init__(self,outcomes=['succeeded','aborted'])
        self._keys = keys
        self.register_input_keys(keys)

    def execute(self,ud):
        for key in self._keys:
            if key not in ud:
                rospy.logerr("Key '%s' not in userdata. Available keys are: %s" % (key, ud.keys()))
                return 'aborted'
        return 'succeeded'

### Test harness
class TestActionlib(unittest.TestCase):
    def test_action_client(self):
        """Test simple action states"""
        sq = Sequence(['succeeded','aborted','preempted','foobar'],'succeeded')

        sq.userdata['g1'] = g1
        sq.userdata['g2'] = g2
        sq.userdata['goal'] = 1
        sq.userdata['goal_alias'] = 1

        with sq:
            # Test single goal policy
            Sequence.add('GOAL_STATIC',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1))
            Sequence.add('GOAL_KEY',
                    SimpleActionState("reference_action",TestAction,
                        goal_key = 'g1'))
            Sequence.add('GOAL_SLOTS',
                    SimpleActionState("reference_action",TestAction,
                        goal_slots = ['goal']))
            Sequence.add('GOAL_SLOTS_REMAP',
                    SimpleActionState("reference_action",TestAction,
                        goal_slots = ['goal']),
                    remapping = {'goal':'goal_alias'})

            # Test goal callback
            def goal_cb_0(ud, default_goal):
                return TestGoal(1)
            Sequence.add('GOAL_CB',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = goal_cb_0))
            Sequence.add('GOAL_CB_LAMBDA',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = lambda ud,goal: TestGoal(1)))
            Sequence.add('GOAL_CB_UD',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = lambda ud,goal: ud.g1,
                        input_keys = ['g1']))
            
            @cb_interface(input_keys=['g1'])
            def goal_cb_1(ud, default_goal):
                return ud.g1
            Sequence.add('GOAL_CB_UD_DECORATOR',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = goal_cb_1))
            Sequence.add('GOAL_CB_ARGS',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = lambda ud,goal,g: TestGoal(g),
                        goal_cb_args = [1]))
            Sequence.add('GOAL_CB_KWARGS',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = lambda ud,goal,gg: TestGoal(gg),
                        goal_cb_kwargs = {'gg':1}))
            Sequence.add('GOAL_CB_ARGS_KWARGS',
                    SimpleActionState("reference_action",TestAction,
                        goal_cb = lambda ud,goal,g,gg: TestGoal(g-gg),
                        goal_cb_args = [2],
                        goal_cb_kwargs = {'gg':1}))

            # Test overriding goal policies
            Sequence.add('GOAL_STATIC_SLOTS',
                    SimpleActionState("reference_action",TestAction,
                        goal = g2,
                        goal_slots = ['goal']))
            Sequence.add('GOAL_STATIC_CB',
                    SimpleActionState("reference_action",TestAction,
                        goal = g2,            
                        goal_cb = CBInterface(
                            lambda ud,goal: setattr(goal,'goal',1),
                            output_keys = ['goal'])))

            # Test result policies
            Sequence.add('RESULT_KEY',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1,
                        result_key = 'res_key'))
            Sequence.add('RESULT_KEY_CHECK', AssertUDState(['res_key']))

            Sequence.add('RESULT_CB',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1,
                        result_cb = CBInterface(
                            lambda ud,res_stat,res: setattr(ud,'res_cb',res),
                            output_keys = ['res_cb'])))
            Sequence.add('RESULT_CB_CHECK', AssertUDState(['res_cb']))

            Sequence.add('RESULT_SLOTS',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1,
                        result_slots = ['result']))
            Sequence.add('RESULT_SLOTS_CHECK', AssertUDState(['result']))

            Sequence.add('RESULT_SLOTS_REMAP',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1,
                        result_slots = ['result']),
                    remapping = {'result':'res_alias'})
            Sequence.add('RESULT_SLOTS_MAP_CHECK', AssertUDState(['res_alias']))

            Sequence.add('RESULT_CB_OUTCOME',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1,
                        result_cb = CBInterface(
                            lambda ud,res_stat,res: 'foobar',
                            outcomes = ['foobar'])))
                    
        sq_outcome = sq.execute()
        assert sq_outcome == 'foobar'

    def test_action_server_wrapper(self):
        """Test action server wrapper."""
        sq = Sequence(['succeeded','aborted','preempted'],'succeeded')
        sq.register_input_keys(['goal','action_goal','action_result'])
        sq.register_output_keys(['action_result'])

        with sq:
            Sequence.add('GOAL_KEY',
                    SimpleActionState("reference_action",TestAction,
                        goal_key = 'action_goal'))
            Sequence.add('GOAL_SLOTS',
                    SimpleActionState("reference_action",TestAction,
                        goal_slots = ['goal']))

            @cb_interface(input_keys=['action_result'],output_keys=['action_result'])
            def res_cb(ud, status, res):
                ud.action_result.result = res.result+1

            Sequence.add('RESULT_CB',
                    SimpleActionState("reference_action",TestAction,
                        goal = g1,
                        result_cb = res_cb))

        asw = ActionServerWrapper(
                'reference_action_sm', TestAction, sq,
                succeeded_outcomes = ['succeeded'],
                aborted_outcomes = ['aborted'],
                preempted_outcomes = ['preempted'],
                expand_goal_slots = True)
        asw.run_server()

        ac = SimpleActionClient('reference_action_sm',TestAction)
        ac.wait_for_server(rospy.Duration(30))

        assert ac.send_goal_and_wait(g1,rospy.Duration(30)) == GoalStatus.SUCCEEDED
        assert ac.get_result().result == 1

    def test_action_preemption(self):
        """Test action preemption"""
        sq = Sequence(['succeeded','aborted','preempted'],'succeeded')

        class SlowRunningState(State):
            def __init__(self):
                State.__init__(self,outcomes=['succeeded','aborted','preempted'])
            def execute(self,ud):
                start_time = rospy.Time.now()
                while rospy.Time.now() - start_time < rospy.Duration(10):
                    rospy.sleep(0.05)
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                return 'succeeded'

        with sq:
            Sequence.add('PREEMPT_ME',SlowRunningState())

        asw = ActionServerWrapper(
                'preempt_action_sm', TestAction, sq,
                succeeded_outcomes = ['succeeded'],
                aborted_outcomes = ['aborted'],
                preempted_outcomes = ['preempted'])
        asw.run_server()

        ac = SimpleActionClient('preempt_action_sm',TestAction)
        ac.wait_for_server(rospy.Duration(30))

        ac.send_goal(g1)
        rospy.sleep(5.0)
        ac.cancel_goal()

        start_time = rospy.Time.now()
        while ac.get_state() == GoalStatus.ACTIVE and rospy.Time.now() - start_time < rospy.Duration(30):
            rospy.sleep(0.5)
        assert ac.get_state() == GoalStatus.PREEMPTED



def main():
  rospy.init_node('smach_actionlib',log_level=rospy.DEBUG)
  rostest.rosrun('smach', 'smach_actionlib', TestActionlib)

if __name__=="__main__":
  main();
