#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from smach import *
from smach_ros import *
from smach_msgs.msg import *

from action_tutorials_interfaces.action import Fibonacci

# Static goals
g1 = Fibonacci.Goal(order=1)  # This goal should succeed

### Test harness
class TestSequence(unittest.TestCase):
    def test_sequence(self):
        """Test adding a sequence of states."""
        node = rclpy.create_node('sequence_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        sq = Sequence(['succeeded','aborted','preempted','done'],connector_outcome='succeeded')
        with sq:
            Sequence.add('FIRST',
                SimpleActionState(node, 'fibonacci', Fibonacci,
                    goal = g1))
            Sequence.add('SECOND',
                SimpleActionState(node, 'fibonacci', Fibonacci,
                    goal = g1))
            Sequence.add('THIRD',
                SimpleActionState(node, 'fibonacci', Fibonacci,
                    goal = g1), {'succeeded':'done'})


        spinner = threading.Thread(target=spin)
        spinner.start()
        outcome = sq.execute()
        assert outcome == 'done'

def main():
    rclpy.init()
    unittest.main()
    rclpy.shutdown()
    
if __name__=="__main__":
    main();
