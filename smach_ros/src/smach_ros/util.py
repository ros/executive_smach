
import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import smach

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
            rospy.loginfo("Received shutdown request... sent preempt... waiting for state machine to terminate.")
            rospy.sleep(1.0)

    ### Add handler
    rospy.core.add_client_shutdown_hook(lambda: handler(sc))

