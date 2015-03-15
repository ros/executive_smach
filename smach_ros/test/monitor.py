#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_ros')
import rospy
import rostest

import unittest

from actionlib import *
from actionlib.msg import *

from std_msgs.msg import Empty

from smach import *
from smach_ros import *

from smach_msgs.msg import *

def pinger():
    pub = rospy.Publisher('/ping', Empty)
    msg = Empty()
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print "publish!"
        pub.publish(msg)
        r.sleep()


def cond_cb(ud, msg):
    """monitor condition cb that modifies user data"""
    assert 'b' in ud
    ud.a = ud.b
    return False

### Test harness
class TestStateMachine(unittest.TestCase):
    def test_userdata(self):
        """Test serial manipulation of userdata."""

        pinger_thread = threading.Thread(target=pinger)
        pinger_thread.start()

        init_ud = UserData()
        init_ud.b = 'A'

        sm = StateMachine(['valid','invalid','preempted'])
        sm.set_initial_state(['MON'],userdata=init_ud)

        assert 'b' in sm.userdata
        assert sm.userdata.b == 'A'

        with sm:
            StateMachine.add(
                'MON',
                MonitorState('/ping', Empty, cond_cb, input_keys=['b'], output_keys=['a']))

        outcome = sm.execute()

        assert outcome == 'invalid'
        assert 'b' in sm.userdata
        assert sm.userdata.b == 'A'
        assert 'a' in sm.userdata
        assert sm.userdata.a == 'A'


def main():
    rospy.init_node('monitor_test',log_level=rospy.DEBUG)
    rostest.rosrun('smach', 'monitor_test', TestStateMachine)

if __name__=="__main__":
    main();

