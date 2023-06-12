#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from smach import *
from smach_ros import *
from smach_msgs.msg import *

# Static goals
from action_tutorials_interfaces.action import Fibonacci
g1 = Fibonacci.Goal(order=1)
g2 = Fibonacci.Goal(order=2)

### Custom state classe
class Setter(RosState):
    """State that sets the key 'a' in its userdata"""
    def __init__(self, node):
        RosState.__init__(self, node,
            outcomes=['done'], output_keys=['a'])
    def execute(self, ud):
        ud.a = 'A'
        self.node.get_logger().info("Added key 'a'.")
        return 'done'

class Getter(RosState):
    """State that grabs the key 'a' from userdata, and sets 'b'"""
    def __init__(self, node):
        RosState.__init__(self, node,
            outcomes=['done'],
            input_keys=['a'], output_keys=['b'])
    def execute(self,ud):
        rate = self.node.create_rate(10)
        while 'a' not in ud:
            self.node.get_logger().info("Waiting for key 'a' to appear. ")
            rate.sleep()
        ud.b = ud.a
        return 'done'

### Test harness
class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.node = rclpy.create_node('state_machine_test')
        self.node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.executor = SingleThreadedExecutor()

    def tearDown(self):
        self.node.destroy_node()

    def spin(self):
        rclpy.spin(self.node, executor=self.executor)

    def test_userdata(self):
        """Test serial manipulation of userdata."""
        sm = StateMachine(['done'])
        with sm:
            StateMachine.add('SETTER', Setter(self.node),{'done':'GETTER'})
            StateMachine.add('GETTER', Getter(self.node),{})

        spinner = threading.Thread(target=self.spin)
        spinner.start()
        outcome = sm.execute()

        assert outcome == 'done'
        assert 'a' in sm.userdata
        assert sm.userdata.a == 'A'
        assert sm.userdata.b == 'A'

    def test_userdata_nesting(self):
        """Test serial manipulation of userdata."""
        sm = StateMachine(['done','preempted','aborted'])
        with sm:
            StateMachine.add('SETTER', Setter(self.node),{'done':'GETTER'})
            StateMachine.add('GETTER', Getter(self.node),{'done':'NEST'})

            sm2 = StateMachine(['done','aborted','preempted'])
            sm2.register_input_keys(['a'])
            with sm2:
                StateMachine.add('ASSERTER', ConditionState(
                    self.node,
                    lambda ud: 'a' in ud,
                    input_keys = ['a']),
                    {'true':'done', 'false':'aborted'})
            StateMachine.add('NEST',sm2)

        spinner = threading.Thread(target=self.spin)
        spinner.start()
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
            StateMachine.add('SETTER', Setter(self.node),{'done':'GETTER'})
            StateMachine.add('GETTER', Getter(self.node),{'done':'NEST'})

            sm2 = StateMachine(['done','aborted','preempted'])
            sm2.register_input_keys(['foo'])
            with sm2:
                StateMachine.add('ASSERTER', ConditionState(
                    self.node,
                    lambda ud: 'foo' in ud,
                    input_keys = ['foo']),
                    {'true':'done','false':'aborted'})
            StateMachine.add('NEST',sm2)

        spinner = threading.Thread(target=self.spin)
        spinner.start()
        outcome = sm.execute()

        assert outcome == 'done'
        assert 'foo' in sm.userdata

    def test_userdata_remapping(self):
        """Test remapping of userdata."""
        sm = StateMachine(['done','preempted','aborted'])
        with sm:
            StateMachine.add('SETTER', Setter(self.node), {'done':'GETTER'}, remapping = {'a':'x'})
            StateMachine.add('GETTER', Getter(self.node), {'done':'done'}, remapping = {'a':'x','b':'y'})

        spinner = threading.Thread(target=self.spin)
        spinner.start()
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
            StateMachine.add_auto('FIRST', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1),['succeeded'])
            StateMachine.add_auto('SECOND', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1),['succeeded'])
            StateMachine.add('THIRD', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1),{'succeeded':'done'})

        spinner = threading.Thread(target=self.spin)
        spinner.start()
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
                StateMachine.add('FIRST', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1), transitions)
                StateMachine.add('SECOND', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g2), transitions)
                StateMachine.add('THIRD', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1), transitions)
        spinner = threading.Thread(target=self.spin)
        spinner.start()
        outcome = sm.execute()

        assert outcome == 'done'

    def test_alt_api(self):
        """Test adding with alt apis."""

        sm = StateMachine(['succeeded','aborted','preempted'])
        with sm.opened():
            sm.add('FIRST', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1),{})
            sm.add('SECOND', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g2),{})
            sm.add('THIRD', SimpleActionState(self.node, 'fibonacci', Fibonacci, goal = g1),{})
        spinner = threading.Thread(target=self.spin)
        spinner.start()
        outcome = sm.execute()

        assert outcome == 'succeeded'
    
def main():
    rclpy.init()
    unittest.main()
    rclpy.shutdown()

if __name__=="__main__":
    main();
