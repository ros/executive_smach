#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading
import traceback

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
                print("Key '%s' not in userdata. Available keys are: %s" % (key, ud.keys()))
                return 'aborted'
        for key in self._keys:
            print("result "+key+": "+str(ud[key]))
        return 'succeeded'


# Create a trivial action server
class FibonacciActionServer(object):
    def __init__(self, node):
        self.__node = node
        self._action_server = ActionServer(
            node,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.__node.get_logger().info('Executing goal...')
        goal_handle.succeed()
        result = Fibonacci.Result()
        return result

# ## Test harness
class TestActionlib(unittest.TestCase):
    def test_action_client(self):
        """Test simple action states"""
        rclpy.init()
        node = rclpy.create_node('simple_action_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        server = FibonacciActionServer(node=node)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        sq = Sequence(['succeeded', 'aborted', 'preempted', 'foobar'], 'succeeded')

        sq.userdata['g1'] = g1
        sq.userdata['g2'] = g2
        sq.userdata['order'] = 1
        sq.userdata['goal_alias'] = 1

        with sq:
            # Test single goal policy
            Sequence.add('GOAL_STATIC',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci, goal=g1))
            Sequence.add('GOAL_KEY',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci, goal_key='g1'))
            Sequence.add('GOAL_SLOTS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci, goal_slots=['order']))
            Sequence.add('GOAL_SLOTS_REMAP',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci, goal_slots=['order']),
                    remapping={'order':'goal_alias'})

            # Test goal callback
            def goal_cb_0(ud, default_goal):
                return Fibonacci.Goal(order=1)
            Sequence.add('GOAL_CB',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=goal_cb_0))
            Sequence.add('GOAL_CB_LAMBDA',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=lambda ud, goal: Fibonacci.Goal(order=1)))
            Sequence.add('GOAL_CB_UD',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=lambda ud, goal: ud.g1,
                        input_keys=['g1']))

            @cb_interface(input_keys=['g1'])
            def goal_cb_1(ud, default_goal):
                return ud.g1
            Sequence.add('GOAL_CB_UD_DECORATOR',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=goal_cb_1))
            Sequence.add('GOAL_CB_ARGS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=lambda ud, goal, g: Fibonacci.Goal(order=g),
                        goal_cb_args=[1]))
            Sequence.add('GOAL_CB_KWARGS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=lambda ud, goal, gg: Fibonacci.Goal(order=gg),
                        goal_cb_kwargs={'gg':1}))
            Sequence.add('GOAL_CB_ARGS_KWARGS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal_cb=lambda ud, goal, g, gg: Fibonacci.Goal(order=(g - gg)),
                        goal_cb_args=[2],
                        goal_cb_kwargs={'gg':1}))

            # Test overriding goal policies
            Sequence.add('GOAL_STATIC_SLOTS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g2,
                        goal_slots=['order']))
            Sequence.add('GOAL_STATIC_CB',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g2,
                        goal_cb=CBInterface(
                            lambda ud, goal: setattr(goal, 'order', 1),
                            output_keys=['goal'])))

            # Test result policies
            Sequence.add('RESULT_KEY',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_key='res_key'))
            Sequence.add('RESULT_KEY_CHECK', AssertUDState(['res_key']))

            Sequence.add('RESULT_CB',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_cb=CBInterface(
                            lambda ud, res_stat, res: setattr(ud, 'res_cb', res),
                            output_keys=['res_cb'])))
            Sequence.add('RESULT_CB_CHECK', AssertUDState(['res_cb']))

            Sequence.add('RESULT_SLOTS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_slots=['sequence']))
            Sequence.add('RESULT_SLOTS_CHECK', AssertUDState(['sequence']))

            Sequence.add('RESULT_SLOTS_REMAP',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_slots=['sequence']),
                    remapping={'sequence': 'res_alias'})
            Sequence.add('RESULT_SLOTS_MAP_CHECK', AssertUDState(['res_alias']))

            Sequence.add('RESULT_CB_OUTCOME',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_cb=CBInterface(
                            lambda ud, res_stat, res: 'foobar',
                            outcomes=['foobar'])))

        spinner = threading.Thread(target=spin)
        spinner.start()
        sq_outcome = sq.execute()
        assert sq_outcome == 'foobar'

def main():
    unittest.main()
    rclpy.shutdown()

if __name__ == "__main__":
    main();
