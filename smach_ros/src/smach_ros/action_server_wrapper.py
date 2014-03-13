import roslib; roslib.load_manifest('smach_ros')
import rospy

import copy
import threading
import traceback

from actionlib.simple_action_server import SimpleActionServer
from smach_msgs.msg import *
import smach

__all__ = ['ActionServerWrapper']

class ActionServerWrapper():
    """SMACH container wrapper with actionlib ActionServer.

    Use this class to associate an action server with a smach
    L{StateMachine<smach.state_machine.StateMachine>}. This allows invocation
    of the state machine over the actionlib API/protocol.

    This class delegates to a provided SMACH container and associates it with an
    action server. The user can specify lists of outcomes which correspond to
    different action result statuses (SUCCEEDED, ABORTED, PREEMPTED). Once the
    delegate state machine leaves one of these outcomes, this wrapper class will
    cause the state machine to terminate, and cause the action server to return
    a result.

    Note that this class does not inherit from L{smach.State<smach.State>} and
    can only be used as a top-level container.
    """

    def __init__(self,
            server_name, action_spec,
            wrapped_container,
            succeeded_outcomes = [],
            aborted_outcomes = [],
            preempted_outcomes = [],
            goal_key = 'action_goal',
            feedback_key = 'action_feedback',
            result_key = 'action_result',
            goal_slots_map = {},
            feedback_slots_map = {},
            result_slots_map = {},
            expand_goal_slots = False,
            pack_result_slots = False
            ):
        """Constructor.

        @type server_name: string
        @param server_name: The name of the action server that this container will
        present.

        @type action_spec: actionlib action msg
        @param action_spec: The type of action this server will present

        @type wrapped_container: L{StateMachine}
        @param wrapped_container: The state machine to manipulate

        @type succeeded_outcomes: array of strings
        @param succeeded_outcomes: Array of terminal state labels which, when left,
        should cause the action server to return SUCCEEDED as a result status.

        @type aborted_outcomes: array of strings
        @param aborted_outcomes: Array of terminal state labels which, when left,
        should cause the action server to return ABORTED as a result status.

        @type preempted_outcomes: array of strings
        @param preempted_outcomes: Array of terminal state labels which, when left,
        should cause the action server to return PREEMPTED as a result status.

        @type goal_key: string
        @param goal_key: The userdata key into which the action goal should be
        stuffed when the action server receives one.

        @type feedback_key: string
        @param feedback_key: The userdata key into which the SMACH container
        can put feedback information relevant to the action.

        @type result_key: string
        @param result_key: The userdata key into which the SMACH container
        can put result information from this action.
        """

        # Store state machine
        self.wrapped_container = wrapped_container
        """State machine that this wrapper talks to."""

        # Register state machine callbacks
        self.wrapped_container.register_transition_cb(self.transition_cb)
        self.wrapped_container.register_termination_cb(self.termination_cb)

        # Grab reference to state machine user data (the user data in the children
        # states scope)
        self.userdata = smach.UserData()

        # Store special userdata keys
        self._goal_key = goal_key
        self._feedback_key = feedback_key
        self._result_key = result_key

        self._goal_slots_map = goal_slots_map
        self._feedback_slots_map = feedback_slots_map
        self._result_slots_map = result_slots_map

        self._expand_goal_slots = expand_goal_slots
        self._pack_result_slots = pack_result_slots

        # Store goal, result, and feedback types
        self.userdata[self._goal_key] = copy.copy(action_spec().action_goal.goal)
        self.userdata[self._result_key] = copy.copy(action_spec().action_result.result)
        self.userdata[self._feedback_key] = copy.copy(action_spec().action_feedback.feedback)

        # Action info
        self._server_name = server_name
        self._action_spec = action_spec

        # Construct action server (don't start it until later)
        self._action_server = SimpleActionServer(
                self._server_name,
                self._action_spec,
                execute_cb=self.execute_cb,
                auto_start=False)

        # Store and check the terminal outcomes
        self._succeeded_outcomes = set(succeeded_outcomes)
        self._aborted_outcomes = set(aborted_outcomes)
        self._preempted_outcomes = set(preempted_outcomes)

        # Make sure the sets are disjoint
        card_of_unions = len(self._succeeded_outcomes | self._aborted_outcomes | self._preempted_outcomes)
        sum_of_cards = (len(self._succeeded_outcomes) + len(self._aborted_outcomes) + len(self._preempted_outcomes))
        if card_of_unions != sum_of_cards:
            rospy.logerr("Succeeded, aborted, and preempted outcome lists were not mutually disjoint... expect undefined behavior.")

    def run_server(self):
        """Run the state machine as an action server.
        Note that this method does not block.
        """

        # Register action server callbacks
        #self._action_server.register_goal_callback(self.goal_cb)
        self._action_server.register_preempt_callback(self.preempt_cb)

        # Stat server (because we disabled auto-start to register the callbacks)
        self._action_server.start()

        rospy.loginfo("Started SMACH action server wrapper, adversiting as '%s'" % self._server_name)

    ### State machine callbacks
    def transition_cb(self, userdata, active_states):
        """Transition callback passed to state machine.
        This method is called each time the state machine transitions.
        """
        rospy.logdebug("Publishing action feedback.")
        # Publish action feedback
        self.publish_feedback(userdata)

    def termination_cb(self, userdata, terminal_states, container_outcome):
        """Termination callback passed to state machine.
        This callback receives the final state and state machine outcome as
        specified by the state-outcome map given to the delegate container
        on construction (see L{ActionServerWrapper.__init__}).

        Remember that in this context, the SMACH container is just a single state
        object, which has an outcome like any other state; it is this outcome on
        which we switch here. This method will determine from the state machine
        outcome which result should be returned to the action client for this goal.
        """
        rospy.logdebug("Wrapped state machine has terminated with final state: "+str(terminal_states)+" and container outcome: "+str(container_outcome))

    def publish_feedback(self, userdata):
        """Publish the feedback message in the userdata db.
        Note that this feedback is independent of smach.
        """
        if self._feedback_key in userdata:
            # This was spewing errors after the fix to ticket #5033 was submitted
            # in the case when _feedback_key is not set
            # For now, the fix is just checking if it exists, and not publishing otherwise
            # The spewage used to not happen because we were looking in self.userdata
            # and the constructor of this class sets the feedback key there to an empty struct
            # TODO figure out what the hell is going on here.
            self._action_server.publish_feedback(userdata[self._feedback_key])
        

    ### Action server callbacks
    def execute_cb(self, goal):
        """Action server goal callback
        This method is called when the action server associated with this state
        machine receives a goal. This puts the goal into the userdata,
        which is the userdata of the contained state.
        """

        # If the state machine is running, we should preempt it before executing it
        # it again.
        rospy.logdebug("Starting wrapped SMACH container") 

        # Accept goal
        #goal = self._action_server.accept_new_goal()

        # Expand the goal into the root userdata for this server
        if self._expand_goal_slots:
            for slot in goal.__slots__:
                self.userdata[slot] = getattr(goal, slot)

        # Store the goal in the container local userdate
        self.userdata[self._goal_key] = goal       

        # Store mapped goal slots in local userdata
        for from_key,to_key in ((k,self._goal_slots_map[k]) for k in self._goal_slots_map):
            self.userdata[to_key] = getattr(goal,from_key)

        # Run the state machine (this blocks)
        try:
            container_outcome = self.wrapped_container.execute(
                    smach.Remapper(
                        self.userdata,
                        self.wrapped_container.get_registered_input_keys(),
                        self.wrapped_container.get_registered_output_keys(),
                        {}))

        except smach.InvalidUserCodeError as ex:
            rospy.logerr("Exception thrown while executing wrapped container.")
            self._action_server.set_aborted()
            return
        except:
            rospy.logerr("Exception thrown:while executing wrapped container: " + traceback.format_exc())
            self._action_server.set_aborted()
            return

        # Grab the (potentially) populated result from the userdata
        result = self.userdata[self._result_key]

        # Store mapped slots in result
        for from_key, to_key in ((k,self._result_slots_map[k]) for k in self._result_slots_map):
            setattr(result, from_key, self.userdata[to_key])

        # If any of the result members have been returned to the parent ud
        # scope, overwrite the ones from the full structure
        if self._pack_result_slots:
            for slot in result.__slots__:
                if slot in self.userdata:
                    setattr(result, slot, self.userdata[slot])

        # Set terminal state based on state machine state outcome
        if container_outcome in self._succeeded_outcomes:
            rospy.loginfo('SUCCEEDED')
            self._action_server.set_succeeded(result)
        elif container_outcome in self._preempted_outcomes:
            rospy.loginfo('PREEMPTED')
            self._action_server.set_preempted(result)
        else: #if container_outcome in self._aborted_outcomes:
            rospy.loginfo('ABORTED')
            self._action_server.set_aborted(result)


    def preempt_cb(self):
        """Action server preempt callback.
        This method is called when the action client preempts an active goal.

        In this case, the StateMachine needs to propagate said preemption to
        the currently active delegate action (the current state).
        """
        rospy.loginfo("Preempt on state machine requested!")
        self.wrapped_container.request_preempt()
