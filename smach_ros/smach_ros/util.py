#!/usr/bin/env python3
import rclpy

import threading
import smach
import time

__all__ = ['set_preempt_handler']

# Signal handler
def set_preempt_handler(sc):
    """Sets a ROS pre-shutdown handler to preempt a given SMACH container when
    ROS receives a shutdown request.

    This can be attached to multiple containers, but only needs to be used on
    the top-level containers.

    @type sc: L{smach.Container}
    @param sc: Container to preempt on ROS shutdown.
    """
    ### Define handler
    def handler(sc):
        sc.request_preempt()

        while sc.is_running():
            rclpy.logging.get_logger(__name__).info("Received shutdown request... sent preempt... waiting for state machine to terminate.")
            time.sleep(1.0)

    ### Add handler
    rclpy.get_default_context().on_shutdown(lambda: handler(sc))
