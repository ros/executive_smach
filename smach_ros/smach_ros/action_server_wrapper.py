import rclpy
from rclpy.node import Node
import copy
import threading
import traceback
import time

import rclpy.action
from rclpy.action.server import GoalStatus, GoalResponse
from rclpy.action import ActionServer,CancelResponse
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

    def __init__(self, node,
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

        self.__node = node
        self.__goal_handle = None
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
        self.userdata[self._goal_key] = action_spec.Goal()
        self.userdata[self._result_key] = action_spec.Result()
        self.userdata[self._feedback_key] = action_spec.Feedback()

        self.wrapped_container.register_input_keys(list([self._goal_key,self._result_key,self._feedback_key]))

        # Action info
        self._server_name = server_name
        self._action_spec = action_spec

        # Construct action server (don't start it until later)

        self._action_server = ActionServer(self.__node,
                                        self._action_spec,
                                        self._server_name,
                                        self.execute_cb,
                                        cancel_callback = self.action_server_wrapper_cancel_callback)
        
        # Store and check the terminal outcomes
        self._succeeded_outcomes = set(succeeded_outcomes)
        self._aborted_outcomes = set(aborted_outcomes)
        self._preempted_outcomes = set(preempted_outcomes)

        # Make sure the sets are disjoint
        card_of_unions = len(self._succeeded_outcomes | self._aborted_outcomes | self._preempted_outcomes)
        sum_of_cards = (len(self._succeeded_outcomes) + len(self._aborted_outcomes) + len(self._preempted_outcomes))
        if card_of_unions != sum_of_cards:
            self.__node.get_logger().error("Succeeded, aborted, and preempted outcome lists were not mutually disjoint... expect undefined behavior.")

    ### State machine callbacks
    def transition_cb(self, userdata, active_states):
        """Transition callback passed to state machine.
        This method is called each time the state machine transitions.
        """
        self.__node.get_logger().debug("Publishing action feedback.")
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
        self.__node.get_logger().debug("Wrapped state machine has terminated with final state: "+str(terminal_states)+" and container outcome: "+str(container_outcome))

    def publish_feedback(self, userdata):
        """Publish the feedback message in the userdata db.
        Note that this feedback is independent of smach.
        """
        if self._feedback_key in userdata and self.__goal_handle is not None:
            # This was spewing errors after the fix to ticket #5033 was submitted
            # in the case when _feedback_key is not set
            # For now, the fix is just checking if it exists, and not publishing otherwise
            # The spewage used to not happen because we were looking in self.userdata
            # and the constructor of this class sets the feedback key there to an empty struct
            # TODO figure out what the hell is going on here.
            self.__goal_handle.publish_feedback(userdata[self._feedback_key])
        


    def action_server_wrapper_cancel_callback(self,cancel_request):
        self.__node.get_logger().debug("cancel callback is called")
        return CancelResponse.ACCEPT


    def preempt_check(self):
        rate = self.__node.create_rate(20)
        while(self.__goal_handle.status not in [GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELING,
                                                GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED]):
            rate.sleep()
            self.__node.get_logger().debug("Status %d" % self.__goal_handle.status)
        self.__node.get_logger().debug("Status %d" % self.__goal_handle.status)
        if self.__goal_handle.is_cancel_requested:
            self.preempt_cb()

    ### Action server callbacks
    def execute_cb(self, goal_handle):
        """Action server goal callback
        This method is called when the action server associated with this state
        machine receives a goal. This puts the goal into the userdata,
        which is the userdata of the contained state.
        """

        # If the state machine is running, we should preempt it before executing it
        # it again.
        self.__goal_handle = goal_handle 
        self.__node.get_logger().debug("Starting wrapped SMACH container") 

        # Expand the goal into the root userdata for this server
        if self._expand_goal_slots:
            for slot in self._action_spec.Goal.get_fields_and_field_types().keys():
                self.userdata[slot] = getattr(goal_handle.request, slot)
                self.wrapped_container.register_input_keys(list([slot]))

        # Store the goal in the container local userdate
        self.userdata[self._goal_key] = goal_handle.request       
        # Store mapped goal slots in local userdata
        for from_key,to_key in ((k,self._goal_slots_map[k]) for k in self._goal_slots_map):
            self.userdata[to_key] = getattr(goal_handle.request,from_key)
            

        thread = threading.Thread(target=self.preempt_check)
        thread.start()

        # Run the state machine (this blocks)
        try:
            container_outcome = self.wrapped_container.execute(
                    smach.Remapper(
                        self.userdata,
                        self.wrapped_container.get_registered_input_keys(),
                        self.wrapped_container.get_registered_output_keys(),
                        {}))

        except smach.InvalidUserCodeError as ex:
            self.__node.get_logger().error("Exception thrown while executing wrapped container.")
            goal_handle.aborted()
            thread.join()
            return
        except:
            self.__node.get_logger().error("Exception thrown:while executing wrapped container: " + traceback.format_exc())
            goal_handle.aborted()
            thread.join()
            return

        # Grab the (potentially) populated result from the userdata
        result = self.userdata[self._result_key]
        # Store mapped slots in result
        for from_key, to_key in ((k,self._result_slots_map[k]) for k in self._result_slots_map):
            setattr(result, from_key, self.userdata[to_key])

        # If any of the result members have been returned to the parent ud
        # scope, overwrite the ones from the full structure
        if self._pack_result_slots:
            for slot in result.get_fields_and_field_types().keys():
                if slot in self.userdata:
                    setattr(result, slot, self.userdata[slot])

        # Set terminal state based on state machine state outcome
        if container_outcome in self._succeeded_outcomes:
            self.__node.get_logger().info('SUCCEEDED')
            goal_handle.succeed()
        elif container_outcome in self._preempted_outcomes:
            self.__node.get_logger().info('PREEMPTED')
            goal_handle.canceled()
        else: #if container_outcome in self._aborted_outcomes:
            self.__node.get_logger().info('ABORTED')
            goal_handle.aborted()
        
        thread.join()
        return result


    def preempt_cb(self):
        """Action server preempt callback.
        This method is called when the action client preempts an active goal.

        In this case, the StateMachine needs to propagate said preemption to
        the currently active delegate action (the current state).
        """
        self.__node.get_logger().info("Preempt on state machine requested!")
        self.wrapped_container.request_preempt()
