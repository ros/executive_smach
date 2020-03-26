#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from std_msgs.msg import Empty

from smach import *
from smach_ros import *
from smach_msgs.msg import *

def pinger():
    node = rclpy.create_node("pinger")
    executor = SingleThreadedExecutor()
    pub = node.create_publisher(Empty, '/ping', 1)
    msg = Empty()
    rate = node.create_rate(10.0)
    while rclpy.ok():
        node.get_logger().info("publish!")
        pub.publish(msg)
        rclpy.spin_once(node, executor=executor)
        rate.sleep()
    node.destroy_node()

def cond_cb(ud, msg):
    """monitor condition cb that modifies user data"""
    assert 'b' in ud
    ud.a = ud.b
    return False

### Test harness
class TestStateMachine(unittest.TestCase):

    def test_userdata(self):
        node = rclpy.create_node('monitor_test')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        executor = SingleThreadedExecutor()
        def spin():
            rclpy.spin(node, executor=executor)

        """Test serial manipulation of userdata."""
        pinger_thread = threading.Thread(target=pinger)
        pinger_thread.start()

        init_ud = UserData()
        init_ud.b = 'A'

        sm = StateMachine(['valid', 'invalid', 'preempted'])
        sm.set_initial_state(['MON'], userdata=init_ud)

        assert 'b' in sm.userdata
        assert sm.userdata.b == 'A'

        with sm:
            StateMachine.add(
                'MON',
                MonitorState(node, '/ping', Empty, cond_cb,
                    input_keys=['b'], output_keys=['a']))

        spinner = threading.Thread(target=spin)
        spinner.start()

        outcome = sm.execute()

        assert outcome == 'invalid'
        assert 'b' in sm.userdata
        assert sm.userdata.b == 'A'
        assert 'a' in sm.userdata
        assert sm.userdata.a == 'A'

        #executor.shutdown()
        #spinner.join()
        node.destroy_node()

def main():
    rclpy.init()
    unittest.main()
    rclpy.shutdown()

if __name__=="__main__":
    main()
