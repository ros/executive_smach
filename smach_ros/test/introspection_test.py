#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor

import threading

import unittest

from smach import StateMachine, UserData
from smach_ros import RosState, IntrospectionServer, IntrospectionClient

### Custom state classe
class Setter(RosState):
    """State that sets the key 'a' in its userdata"""
    def __init__(self, node):
        RosState.__init__(self, node, outcomes=['done'], output_keys=['a'])
    def execute(self,ud):
        ud.a = 'A'
        self.node.get_logger().info("Added key 'a'.")
        return 'done'

class Getter(RosState):
    """State that grabs the key 'a' from userdata, and sets 'b'"""
    def __init__(self, node):
        RosState.__init__(self, node, outcomes=['done'], input_keys=['a'], output_keys=['b'])
    def execute(self,ud):
        rate = self.node.create_rate(10)
        while 'a' not in ud and rclpy.ok():
            self.node.get_logger().info("Waiting for key 'a' to appear. ")
            rate.sleep()
        ud.b = ud.a
        return 'done'

### Test harness
class TestIntrospection(unittest.TestCase):

    def test_introspection(self):
        """Test introspection system."""
        node = rclpy.create_node("sm_node")
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()

        # Construct state machine
        sm = StateMachine(['done'])
        sm2 = StateMachine(['done'])
        sm3 = StateMachine(['done'])

        with sm:
            # Note: the following "Getter" state should fail
            StateMachine.add('GETTER1', Getter(node), {})
            StateMachine.add('SM2', sm2, {'done':'SM3'})
            with sm2:
                StateMachine.add("SETTER", Setter(node), {})
            StateMachine.add('SM3', sm3, {'done':'done'})
            with sm3:
                StateMachine.add("SETTER", Setter(node), {})
            StateMachine.add('GETTER2', Getter(node), {'done':'SM2'})

        sm.set_initial_state(['GETTER1'])
        sm2.set_initial_state(['SETTER'])
        sm3.set_initial_state(['SETTER'])

        # Run introspector
        intro_server = IntrospectionServer('intro_test', sm, '/intro_test')
        intro_server.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        intro_server.start()

        intro_client = IntrospectionClient()
        intro_client.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        servers = intro_client.get_servers()

        executor.add_node(node)
        executor.add_node(intro_server)
        executor.add_node(intro_client)

        def spin(executor):
            executor.spin()

        spinner = threading.Thread(target=spin, args=(executor,))
        spinner.start()

        rate = intro_client.create_rate(10)
        while '/intro_test' not in servers and rclpy.ok():
            servers = intro_client.get_servers()
            intro_client.get_logger().info("Smach servers: "+str(servers))
            rate.sleep()

        assert '/intro_test' in servers

        # Set initial state
        injected_ud = UserData()
        injected_ud.a = 'A'
        init_set = intro_client.set_initial_state('intro_test',
            '/intro_test',
            ['SM2'],
            injected_ud,
            timeout = rclpy.time.Duration(seconds=10.0))

        assert init_set

        outcome = sm.execute()

        assert outcome == 'done'

        intro_server.stop()
        node.get_logger().info("Server stopped")
        intro_client.destroy_node()
        node.get_logger().info("Client destroyed")
        intro_server.destroy_node()
        node.get_logger().info("Server destroyed")

        executor.shutdown()
        node.destroy_node()
        spinner.join()


def main():
    rclpy.init()
    unittest.main()
    rclpy.shutdown()

if __name__=="__main__":
    main()
