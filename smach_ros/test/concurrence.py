#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from smach import *
from smach_ros import *
from smach_msgs.msg import *

### Custom tate classe
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
            outcomes=['done','preempted'],
            input_keys=['a'], output_keys=['b'])
    def execute(self, ud):
        rate = self.node.create_rate(10)
        while 'a' not in ud:
            self.node.get_logger().info("Waiting for key 'a' to appear. ")
            rate.sleep()
        ud.b = ud.a
        rate.sleep()
        if self.preempt_requested():
            return 'preempted'
        return 'done'

### Test harness
class TestConcurrence(unittest.TestCase):
    def test_concurrence(self):
        """Test concurrent container."""
        node = rclpy.create_node('sequence_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        sm = StateMachine(['done','succeeded'])
        with sm:
            cc = Concurrence(['succeeded','done'],
                    default_outcome = 'done',
                    outcome_map = {'succeeded':{'SETTER':'done'}})
            sm.add('CONCURRENT',cc)
            with cc:
                Concurrence.add('SETTER', Setter(node))
                Concurrence.add('GETTER', Getter(node))

        spinner = threading.Thread(target=spin)
        spinner.start()
        outcome = sm.execute()

        assert outcome == 'succeeded'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

        node.destroy_node()

class TestPreempt(unittest.TestCase):
    def test_preempt(self):
        """Test concurrent container that preempts siblings."""
        node = rclpy.create_node('sequence_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        cc = Concurrence(['succeeded','done'],
                default_outcome = 'done',
                child_termination_cb = lambda so: True,
                outcome_map = {'succeeded':{'SETTER':'done', 'GETTER':'preempted'}})
        with cc:
            Concurrence.add('SETTER', Setter(node))
            Concurrence.add('GETTER', Getter(node))

        spinner = threading.Thread(target=spin)
        spinner.start()
        outcome = cc.execute()

        assert outcome == 'succeeded'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

        node.destroy_node()

class TestNoPreempt(unittest.TestCase):
    def test_no_preempt(self):
        """Test concurrent container that doesnt preempt siblings."""
        node = rclpy.create_node('sequence_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        cc = Concurrence(['succeeded','done'],
                default_outcome = 'done',
                child_termination_cb = lambda so: False,
                outcome_map = {
                    'succeeded':{
                        'SETTER':'done',
                        'GETTER':'done'}})
        with cc:
            Concurrence.add('SETTER', Setter(node))
            Concurrence.add('GETTER', Getter(node))

        spinner = threading.Thread(target=spin)
        spinner.start()
        outcome = cc.execute()

        assert outcome == 'succeeded'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

        node.destroy_node()

class TestOutcome(unittest.TestCase):
    def test_outcome_cb(self):
        """Test concurrent container that doesnt preempt siblings."""
        node = rclpy.create_node('sequence_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        cc = Concurrence(['succeeded','done'],
                default_outcome = 'done',
                child_termination_cb = lambda so: False,
                outcome_cb = lambda so: list(set(so.values()))[0])
        with cc:
            Concurrence.add('SETTER', Setter(node))
            Concurrence.add('GETTER', Getter(node))

        spinner = threading.Thread(target=spin)
        spinner.start()
        outcome = cc.execute()

        assert outcome == 'done'
        assert 'a' in cc.userdata
        assert 'b' in cc.userdata
        assert cc.userdata.a == 'A'
        assert cc.userdata.b == 'A'

        node.destroy_node()

def main():
    rclpy.init()
    unittest.main()
    rclpy.shutdown()

if __name__=="__main__":
    main();
