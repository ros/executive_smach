#!/usr/bin/env python3
import rclpy

import threading
from functools import partial

from .ros_state import RosState

__all__ = ['MonitorState']

class MonitorState(RosState):
    """
    A state that will check a given ROS topic with a condition function.
    """
    def __init__(self, node, topic, msg_type, cond_cb, max_checks=-1, input_keys = [], output_keys=[]):
        """State constructor
        @type node rclpy.node.Node
        @param node the ROS2 node executing the state

        @type topic string
        @param topic the topic to monitor

        @type msg_type a ROS message type
        @param msg_type determines the type of the monitored topic

        @type max_checks int
        @param max_checks the number of messages to receive and evaluate. If cond_cb returns False for any
               of them, the state will finish with outcome 'invalid'. If cond_cb returns True for
               all of them, the outcome will be 'valid'

        """
        RosState.__init__(
            self,
            node,
            outcomes=['valid','invalid','preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._topic = topic
        self._msg_type = msg_type
        self._cond_cb = cond_cb
        self._max_checks = max_checks
        self._n_checks = 0

        self._trigger_event = threading.Event()

        self._sub = self.node.create_subscription(self._msg_type,
            self._topic, self._cb, 1)

    def execute(self, ud):
        # If prempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        self._n_checks = 0
        self._ud = ud
        self._trigger_event.clear()

        self._trigger_event.wait()
        self._ud = None

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self._max_checks > 0 and self._n_checks >= self._max_checks:
            return 'valid'

        return 'invalid'

    def _cb(self, msg) :
        if self._ud is None: return

        self.node.get_logger().debug("MonitorState._cb: {}, {}".format(self._ud, msg))
        try:
            if self._cond_cb(self._ud, msg):
                self._n_checks +=1
            else:
                self._trigger_event.set()
        except Exception as e:
            self.node.get_logger().error("Error thrown while executing condition callback %s: %s" % (str(self._cond_cb), e))
            self._trigger_event.set()

        if (self._max_checks > 0 and self._n_checks >= self._max_checks):
            self._trigger_event.set()

    def request_preempt(self):
        RosState.request_preempt(self)
        self._trigger_event.set()
