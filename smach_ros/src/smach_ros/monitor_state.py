
import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback

import smach

__all__ = ['MonitorState']

class MonitorState(smach.State):
    """A state that will check a given ROS topic with a condition function. 
    """
    def __init__(self, topic, msg_type, cond_cb, max_checks=-1):
        smach.State.__init__(self,outcomes=['valid','invalid','preempted'])

        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = cond_cb
        self._max_checks = max_checks
        self._n_checks = 0

        self._trigger_cond = threading.Condition()

    def execute(self, ud):
        self._n_checks = 0

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=[ud])

        self._trigger_cond.acquire()
        self._trigger_cond.wait()
        self._trigger_cond.release()

        self._sub.unregister()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self._max_checks > 0 and self._n_checks >= self._max_checks:
            return 'valid'

        return 'invalid'

    def _cb(self, msg, ud):
        self._n_checks += 1
        try:
            if (self._max_checks > 0 and self._n_checks >= self._max_checks) or not self._cond_cb(ud, msg):
                self._trigger_cond.acquire()
                self._trigger_cond.notify()
                self._trigger_cond.release()
        except:
            rospy.logerr("Error thrown while executing condition callback %s" % str(self._cond_cb))
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()
