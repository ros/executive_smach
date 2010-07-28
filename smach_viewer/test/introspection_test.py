#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_viewer')
import rospy
import rostest

import unittest

from actionlib import *
from actionlib.msg import *

from smach import *

from smach_msgs.msg import *
from geometry_msgs.msg import PoseStamped


# Static goals
g1 = TestGoal(1) # This goal should succeed
g2 = TestGoal(2) # This goal should abort
g3 = TestGoal(3) # This goal should be rejected

### Custom tate classe
class Setter(State):
  """State that sets the key 'a' in its userdata"""
  def __init__(self):
    State.__init__(self,default_outcome='done')
  def enter(self):
    rospy.sleep(0.5)
    self.userdata.a = 'A'
    self.userdata.a_message = PoseStamped()
    rospy.loginfo("Added key 'a'.")
    rospy.sleep(0.5)
    return 'done'

class Getter(State):
  """State that grabs the key 'a' from userdata, and sets 'b'"""
  def __init__(self):
    State.__init__(self,default_outcome='done')
  def enter(self):
    rospy.sleep(0.5)
    while 'a' not in self.userdata:
      #rospy.loginfo("Waiting for key 'a' to appear. ")
      rospy.sleep(0.1)
    self.userdata.b = self.userdata.a
    return 'done'

### Test harness
class TestStateMachine(unittest.TestCase):
  def test_introspection(self):
    """Test introspection system."""
    # Construct state machine
    sm = StateMachine(['aborted','preempted'])
    sm2 = StateMachine(['done'])
    sm3 = StateMachine(['done'])
    sm4 = StateMachine(['done'])
    sm5 = StateMachine(['done'])


    con_split = ConcurrentSplit(default_outcome = 'succeeded')
    con_split.add(
        ('SETTER', Setter()),
        ('RADICAL',
          sm5.add(
            ('T6',SPAState(),
              { 'succeeded':'SETTER',
                'aborted':'T6',
                'preempted':'SETTER'}),
            ('SETTER',Setter(),
              { 'done':'done'}) ) ),
        ('GETTER', Getter())
        )
    con_split.add_outcome_map(({'SETTER':'done'},'succeeded'))


    sm.add(
        state_machine.sequence('done',
          ('GETTER1', Getter(), {}),
          ('S2',
            sm2.add(
              ('SETTER', Setter(), {'done':'A_SPLIT'}),
              ('A_SPLIT', con_split, {'succeeded':'done'}) ),
            {} ),
          ('S3',
            sm3.add(
              ('SETTER', Setter(), {'done':'RADICAL'}),
              ('RADICAL',
                sm4.add(
                  ('T5',SPAState(),
                    { 'succeeded':'SETTER',
                      'aborted':'T5',
                      'preempted':'SETTER'}),
                    ('SETTER',Setter(),{'done':'done'})
                  ),
                {'done':'SETTER2'} ),
              ('SETTER2', Setter(), {'done':'done'})
              ),
            {'done':'TRINARY!'} )
          )
        )

    sm.add(('TRINARY!',SPAState(),{'succeeded':'T2','aborted':'T3','preempted':'T4'}))
    sm.add(
        state_machine.sequence('succeeded',
          ('T2',SPAState(),{}),
          ('T3',SPAState(),{'aborted':'S2'}),
          ('T4',SPAState(),{'succeeded':'GETTER2','aborted':'TRINARY!'})
          )
        )

    sm.add(('GETTER2', Getter(), {'done':'GETTER1'}))

    # Set default initial states
    sm.set_initial_state(['GETTER1'],smach.UserData())
    sm2.set_initial_state(['SETTER'],smach.UserData())
    sm3.set_initial_state(['SETTER'],smach.UserData())
    sm4.set_initial_state(['T5'],smach.UserData())
    sm5.set_initial_state(['T6'],smach.UserData())

    # Run introspector
    intro_server = smach.IntrospectionServer('intro_test',sm,'/intro_test')

    intro_client = smach.IntrospectionClient()
    servers = intro_client.get_servers()

    rospy.loginfo("Smach servers: "+str(servers))
    assert '/intro_test' in servers

    # Set initial state
    injected_ud = smach.UserData()
    injected_ud.a = 'A'
    init_set = intro_client.set_initial_state('/intro_test','/intro_test',['S2'],injected_ud,timeout = rospy.Duration(10.0))

    assert init_set

    sm.enter()

    assert sm.get_outcome() == 'done'




def main():
  rospy.init_node('introspection_test',log_level=rospy.DEBUG)
  rostest.rosrun('smach_viewer', 'state_machine_test', TestStateMachine)

if __name__=="__main__":
  main();
