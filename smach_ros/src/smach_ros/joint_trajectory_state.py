
import roslib; roslib.load_manifest('smach_ros')
import rospy

from smach.state import State
from smach_ros.simple_action_state import SimpleActionState

from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

__all__ = ['JointTrajectoryState']

def reset_timestamp(ud, goal):
    rospy.logdebug("Restting timestamp in joint trajectory goal")
    goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(0.2)
    return goal

class JointTrajectoryState(SimpleActionState):
    def __init__(self,
            controller_name,
            trajectory_name,
            exec_timeout = rospy.Duration(60.0),
            preempt_timeout = rospy.Duration(60.0)):
        """Constructor for JointTrajectoryState.

        @type controller_name: string
        @param controller_name: The name of the joint trajectory controller.

        @type trajectory_name: string
        @param trajectory_name: The name of the trajectory.

        @type exec_timeout: C{rospy.Duration}
        @param exec_timeout: This is the timeout used for sending a preempt message
        to the delegate action. This is C{None} by default, which implies no
        timeout. 

        @type preempt_timeout: C{rospy.Duration}
        @param preempt_timeout: This is the timeout used for aborting after a
        preempt has been sent to the action and no result has been received. This
        timeout begins counting after a preempt message has been sent.
        """

        # Get trajectory joints and waypoints
        joint_names = rospy.get_param(trajectory_name +"/joint_names")
        waypoints = rospy.get_param(trajectory_name +"/waypoints")

        # Construct goal objects
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names

        # Addpoints to trajectory
        for waypoint in waypoints:
            goal.trajectory.points.append(
                    JointTrajectoryPoint(
                        positions = waypoint[0],
                        velocities = waypoint[1],
                        accelerations = waypoint[2],
                        time_from_start = rospy.Duration(waypoint[3])))

        SimpleActionState.__init__(self,
                controller_name+'/joint_trajectory_generator',JointTrajectoryAction,
                goal = goal,
                goal_cb = reset_timestamp,
                exec_timeout = exec_timeout,
                preempt_timeout = preempt_timeout)

