#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from std_srvs.srv import Empty

from smach import *
from smach_ros import *
from smach_msgs.msg import *

def server():
    node = rclpy.create_node("test_server")
    executor = SingleThreadedExecutor()

    def empty_server(req, res):
        node.get_logger().info("Service called!")
        res = Empty.Response()
        return res

    service = node.create_service(
            Empty,
            '/empty',
            empty_server)
    rclpy.spin(node, executor=executor)

### Test harness
class TestServices(unittest.TestCase):
    def test_service_cb(self):
        """Test calling a service with a callback."""

        node = rclpy.create_node('service_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        sm = StateMachine(['succeeded','aborted','preempted','done'])
        with sm:
            def foo_response_cb(userdata, response):
                userdata.foo_var_out = 'foo!'
                return 'succeeded'

            StateMachine.add('FOO',
                ServiceState(node,
                    '/empty',
                    Empty,
                    response_cb=foo_response_cb,
                    output_keys=['foo_var_out']),
                    remapping={'foo_var_out':'sm_var'},
                    transitions={'succeeded':'done'})

        spinner = threading.Thread(target=spin)
        spinner.start()
        outcome = sm.execute()

        node.get_logger().warn("OUTCOME: "+outcome)

        assert outcome == 'done'

        node.destroy_node()

def main():
    rclpy.init()
    server_thread = threading.Thread(target=server)
    server_thread.start()
    unittest.main()
    rclpy.shutdown()

if __name__=="__main__":
    main();
