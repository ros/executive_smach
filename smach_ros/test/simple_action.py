#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from action_tutorials_interfaces.action import Fibonacci

from smach import *
from smach_ros import *
from smach_msgs.msg import *

# Static goals
g1 = Fibonacci.Goal(order=1)  # This goal should succeed
g2 = Fibonacci.Goal(order=2)  # This goal should abort
g3 = Fibonacci.Goal(order=3)  # This goal should be rejected


class AssertUDState(State):
    def __init__(self, keys):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self._keys = keys
        self.register_input_keys(keys)

    def execute(self, ud):
        for key in self._keys:
            if key not in ud:
                rospy.logerr("Key '%s' not in userdata. Available keys are: %s" % (key, ud.keys()))
                return 'aborted'
        return 'succeeded'


# ## Test harness
class TestActionlib(unittest.TestCase):
    def test_action_client(self):
        """Test simple action states"""
        sq = Sequence(['succeeded', 'aborted', 'preempted', 'foobar'], 'succeeded')

        sq.userdata['g1'] = g1
        sq.userdata['g2'] = g2
        sq.userdata['goal'] = 1
        sq.userdata['goal_alias'] = 1

        with sq:
            # Test single goal policy
            Sequence.add('GOAL_STATIC',
                    SimpleActionState(
                        "reference_action", TestAction, goal=g1))
            Sequence.add('GOAL_KEY',
                    SimpleActionState(
                        "reference_action", TestAction, goal_key='g1'))
            Sequence.add('GOAL_SLOTS',
                    SimpleActionState(
                        "reference_action", TestAction, goal_slots=['goal']))
            Sequence.add('GOAL_SLOTS_REMAP',
                    SimpleActionState(
                        "reference_action", TestAction, goal_slots=['goal']),
                    remapping={'goal':'goal_alias'})

            # Test goal callback
            def goal_cb_0(ud, default_goal):
                return TestGoal(1)
            Sequence.add('GOAL_CB',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=goal_cb_0))
            Sequence.add('GOAL_CB_LAMBDA',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=lambda ud, goal: TestGoal(1)))
            Sequence.add('GOAL_CB_UD',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=lambda ud, goal: ud.g1,
                        input_keys=['g1']))

            @cb_interface(input_keys=['g1'])
            def goal_cb_1(ud, default_goal):
                return ud.g1
            Sequence.add('GOAL_CB_UD_DECORATOR',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=goal_cb_1))
            Sequence.add('GOAL_CB_ARGS',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=lambda ud, goal, g: TestGoal(g),
                        goal_cb_args=[1]))
            Sequence.add('GOAL_CB_KWARGS',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=lambda ud, goal, gg: TestGoal(gg),
                        goal_cb_kwargs={'gg':1}))
            Sequence.add('GOAL_CB_ARGS_KWARGS',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal_cb=lambda ud, goal, g, gg: TestGoal(g - gg),
                        goal_cb_args=[2],
                        goal_cb_kwargs={'gg':1}))

            # Test overriding goal policies
            Sequence.add('GOAL_STATIC_SLOTS',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g2,
                        goal_slots=['goal']))
            Sequence.add('GOAL_STATIC_CB',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g2,
                        goal_cb=CBInterface(
                            lambda ud, goal: setattr(goal, 'goal', 1),
                            output_keys=['goal'])))

            # Test result policies
            Sequence.add('RESULT_KEY',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g1,
                        result_key='res_key'))
            Sequence.add('RESULT_KEY_CHECK', AssertUDState(['res_key']))

            Sequence.add('RESULT_CB',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g1,
                        result_cb=CBInterface(
                            lambda ud, res_stat, res: setattr(ud, 'res_cb', res),
                            output_keys=['res_cb'])))
            Sequence.add('RESULT_CB_CHECK', AssertUDState(['res_cb']))

            Sequence.add('RESULT_SLOTS',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g1,
                        result_slots=['result']))
            Sequence.add('RESULT_SLOTS_CHECK', AssertUDState(['result']))

            Sequence.add('RESULT_SLOTS_REMAP',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g1,
                        result_slots=['result']),
                    remapping={'result': 'res_alias'})
            Sequence.add('RESULT_SLOTS_MAP_CHECK', AssertUDState(['res_alias']))

            Sequence.add('RESULT_CB_OUTCOME',
                    SimpleActionState(
                        "reference_action", TestAction,
                        goal=g1,
                        result_cb=CBInterface(
                            lambda ud, res_stat, res: 'foobar',
                            outcomes=['foobar'])))

        sq_outcome = sq.execute()
        assert sq_outcome == 'foobar'

    def test_action_client_timeout(self):
        """Test simple action state server timeout"""
        sq = Sequence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        sq.userdata['g1'] = g1

        with sq:
            # Test single goal policy
            Sequence.add(
                'GOAL_STATIC',
                SimpleActionState(
                    "reference_action_not_available", TestAction,
                    goal=g1,
                    server_wait_timeout=rospy.Duration(1.0)))

        sq_outcome = sq.execute()


def main():
    rospy.init_node('smach_actionlib', log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'smach_actionlib', TestActionlib)

if __name__ == "__main__":
    main();
