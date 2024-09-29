SMACH
=====

SMACH is a task-level python execution framework for rapidly composing complex
robot behaviors.

![travis](https://travis-ci.org/jbohren/executive_smach.svg?branch=master)

# Example 

```python
import smach
import rclpy
from rclpy.node import Node

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rclpy.logging.get_logger("foo").info('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rclpy.logging.get_logger("bar").info('Executing state BAR')
        return 'outcome2'
        


class SmachExampleStateMachine(Node):

    def __init__(self):
        super().__init__('smach_example_state_machine')

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('FOO', Foo(), 
                                transitions={'outcome1':'BAR', 
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', Bar(), 
                                transitions={'outcome2':'FOO'})

        # Execute SMACH plan
        outcome = sm.execute()


# main
def main(args=None):
    rclpy.init(args=args)

    smach_example_state_machine = SmachExampleStateMachine()

    rclpy.spin(smach_example_state_machine)

    smach_example_state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
