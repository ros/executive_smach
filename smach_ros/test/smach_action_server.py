import rclpy
import rclpy.time
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import unittest
import threading

from smach import *
from smach_ros import *

from rclpy.action.server import GoalStatus
from rclpy.action import ActionClient

from action_msgs.srv import CancelGoal


from action_tutorials_interfaces.action import Fibonacci
from smach_msgs.msg import *



# Static goals
g1 = Fibonacci.Goal(order=1)  # This goal should succeed
g2 = Fibonacci.Goal(order=2)  # This goal should abort
g3 = Fibonacci.Goal(order=3)  # This goal should be rejected

class AssertUDState(RosState):
    def __init__(self,node,keys):
        RosState.__init__(self,node,outcomes=['succeeded', 'aborted'])
        self._keys = keys
        self.register_input_keys(keys)

    def execute(self, ud):
        for key in self._keys:
            if key not in ud:
                self.node.get_logger().error("Key '%s' not in userdata. Available keys are: %s" % (key, ud.keys()))
                return 'aborted'
        return 'succeeded'

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
        result.sequence.append(0)
        return result


# ## Test harness
class TestActionlib(unittest.TestCase):
    
    
    def test_action_client(self):
        """Test simple action states"""
        
        node = rclpy.create_node('test_action_client')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        node.get_logger().info("")
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
            Sequence.add('RESULT_KEY_CHECK', AssertUDState(node,['res_key']))

            Sequence.add('RESULT_CB',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_cb=CBInterface(
                            lambda ud, res_stat, res: setattr(ud, 'res_cb', res),
                            output_keys=['res_cb'])))
            Sequence.add('RESULT_CB_CHECK', AssertUDState(node,['res_cb']))

            Sequence.add('RESULT_SLOTS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_slots=['sequence']))
            Sequence.add('RESULT_SLOTS_CHECK', AssertUDState(node,['sequence']))

            Sequence.add('RESULT_SLOTS_REMAP',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_slots=['sequence']),
                    remapping={'sequence': 'res_alias'})
            Sequence.add('RESULT_SLOTS_MAP_CHECK', AssertUDState(node,['res_alias']))

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

    def test_action_server_wrapper(self):
        """Test action server wrapper."""
        node = rclpy.create_node('smach_action_server_wrapper_test_server')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        node.get_logger().info("test_action_server_wrapper")
        executor = MultiThreadedExecutor(2)
        server = FibonacciActionServer(node=node)
        def spin():
            rclpy.spin(node, executor=executor)

        sq = Sequence(['succeeded', 'aborted', 'preempted'], 'succeeded')
        with sq:
            Sequence.add('GOAL_KEY',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci, goal_key='action_goal'))
            Sequence.add('GOAL_SLOTS',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci, goal_slots=['order']))

            @cb_interface(input_keys=['action_result'], output_keys=['action_result'])
            def res_cb(ud, status, res):
                res.sequence.append(1)
                ud.action_result.sequence = res.sequence

            Sequence.add('RESULT_CB',
                    SimpleActionState(node,
                        "fibonacci", Fibonacci,
                        goal=g1,
                        result_cb=res_cb))

        asw = ActionServerWrapper(node,
                'reference_action_sm', Fibonacci, sq,
                succeeded_outcomes=['succeeded'],
                aborted_outcomes=['aborted'],
                preempted_outcomes=['preempted'],
                expand_goal_slots=True)
        
        spinner = threading.Thread(target=spin)
        spinner.start()

        ac = ActionClient(node,Fibonacci,"reference_action_sm")
        server_ready = ac.wait_for_server(rclpy.time.Duration(seconds = 30))

        future = ac.send_goal_async(g1, rclpy.time.Duration(seconds = 30))

        start_time = node.get_clock().now()
        while node.get_clock().now() - start_time < rclpy.time.Duration(seconds=30.0):
            if future.result() is not None:
                if future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    break

        assert future.result().status == GoalStatus.STATUS_SUCCEEDED
        assert future.result().get_result().result.sequence[-1] == 1
        
    
    

    def test_action_preemption(self):
        """Test action preemption"""

        node = rclpy.create_node('test_action_preemption')
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        node.get_logger().info("test_action_preemption")
        executor = MultiThreadedExecutor(3)
        def spin():
            rclpy.spin(node, executor=executor)

        sq = Sequence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        class SlowRunningState(RosState):
            def __init__(self,node):
                RosState.__init__(self,node,outcomes=['succeeded', 'aborted', 'preempted'])

            def execute(self, ud):
                start_time = self.node.get_clock().now()
                self.node.get_logger().info("Executing SlowRunningState")
                rate = self.node.create_rate(20)
                while self.node.get_clock().now() - start_time < rclpy.time.Duration(seconds=30.0):
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                return 'succeeded'

        with sq:
            Sequence.add('PREEMPT_ME', SlowRunningState(node))

        asw = ActionServerWrapper(node,
                'preempt_action_sm', Fibonacci, sq,
                succeeded_outcomes=['succeeded'],
                aborted_outcomes=['aborted'],
                preempted_outcomes=['preempted'])
        

        ac = ActionClient(node,Fibonacci,"preempt_action_sm")

        def done__callback(future):
            handle = future.result()    
            new_future = ac._cancel_goal(handle)

        ac.wait_for_server(rclpy.time.Duration(seconds=30.0))
        future = ac.send_goal_async(g1)
        future.add_done_callback(done__callback)

        spinner = threading.Thread(target=spin)
        spinner.start()

        start_time = node.get_clock().now()
        while node.get_clock().now() - start_time < rclpy.time.Duration(seconds=30.0):
            if future.result() is not None:
                if  future.result().status == GoalStatus.STATUS_CANCELED or future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    break
        assert future.result().status == GoalStatus.STATUS_CANCELED
        
        return 
        
        
    
    def test_action_client_timeout(self):
        node = rclpy.create_node('test_action_client_timeout')
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
                    "fibonacci",
                    Fibonacci,
                    goal=Fibonacci.Goal(order=1),
                    server_wait_timeout=rclpy.time.Duration(seconds=5.0)))

        spinner = threading.Thread(target=spin)
        spinner.start()
        sq_outcome = sm.execute()
        return
    
def main(args = None):
    rclpy.init(args = args)
    unittest.main()
    rclpy.shutdown()

if __name__ == "__main__":
    main();
