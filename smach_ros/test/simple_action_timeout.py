#!/usr/bin/env python3
import rclpy
import rclpy.time
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from action_tutorials_interfaces.action import Fibonacci

from smach import *
from smach_ros import *
from smach_msgs.msg import *

# ## Test harness
class TestActionlib(unittest.TestCase):

    def test_action_client_timeout(self):
        node = rclpy.create_node('simple_action_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        """Test simple action state server timeout"""
        sm = StateMachine(['succeeded', 'aborted', 'preempted'])

        with sm:
            # Test single goal policy
            StateMachine.add(
                'GOAL_STATIC',
                SimpleActionState(
                    node,
                    "reference_action_not_available",
                    Fibonacci,
                    goal=Fibonacci.Goal(order=1),
                    server_wait_timeout=rclpy.time.Duration(seconds=10.0)))

        spinner = threading.Thread(target=spin)
        spinner.start()
        sq_outcome = sm.execute()

def main():
    rclpy.init()
    unittest.main()
    rclpy.shutdown()

if __name__ == "__main__":
    main();
