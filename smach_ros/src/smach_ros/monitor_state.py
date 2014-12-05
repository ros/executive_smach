
import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback

import smach

__all__ = ['MonitorState']

class MonitorState(smach.State):
    """
    A state that will check a given ROS topic with a condition function.
    """
    def __init__(self, topic, msg_type, cond_cb, max_checks=-1,input_keys = [],output_keys=[]):
        """State constructor
        @type topic string
        @param topic the topic to monitor

        @type msg_type a ROS message type
        @param msg_type determines the type of the monitored topic

        @type max_checks int
        @param max_checks the number of messages to receive and evaluate. If cond_cb returns False for any
               of them, the state will finish with outcome 'invalid'. If cond_cb returns True for 
               all of them, the outcome will be 'valid'
        
        """
        smach.State.__init__(
            self,
            outcomes=['valid','invalid','preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = cond_cb
        self._max_checks = max_checks
        self._n_checks = 0

        self._trigger_event = threading.Event()

    def execute(self, ud):
        # If prempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self._n_checks = 0
        self._trigger_event.clear()

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=ud)

        self._trigger_event.wait()
        self._sub.unregister()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self._max_checks > 0 and self._n_checks >= self._max_checks:
            return 'valid'

        return 'invalid'

    def _cb(self,msg,ud) :
        try:
            if self._cond_cb(ud, msg):
                self._n_checks +=1
            else:
                self._trigger_event.set()
        except Exception as e:
            rospy.logerr("Error thrown while executing condition callback %s: %s" % (str(self._cond_cb), e))
            self._trigger_event.set()
            
        if (self._max_checks > 0 and self._n_checks >= self._max_checks):
            self._trigger_event.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
