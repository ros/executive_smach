#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
import rclpy.action
from rclpy.action.client import GoalStatus
from rclpy.task import Future
from enum import Enum

import threading
import traceback

import time

import smach
from .ros_state import RosState

__all__ = ['SimpleActionState']


class ActionState(Enum):
    # Meta-states for this action
    WAITING_FOR_SERVER = 0
    INACTIVE = 1
    ACTIVE = 2
    CANCELING = 3
    CANCELED = 4
    COMPLETED = 5


class SimpleActionState(RosState):
    """Simple action client state.
    Use this class to represent an ActionClient as a state in a state machine.
    """

    def __init__(self,
            node,
            # Action info
            action_name,
            action_spec,
            # Default goal
            goal = None,
            goal_key = None,
            goal_slots = [],
            goal_cb = None,
            goal_cb_args = [],
            goal_cb_kwargs = {},
            # Result modes
            result_key = None,
            result_slots = [],
            result_cb = None,
            result_cb_args = [],
            result_cb_kwargs = {},
            # Keys
            input_keys = [],
            output_keys = [],
            outcomes = ['succeeded', 'aborted', 'preempted'],
            # Timeouts
            exec_timeout: Duration = None,
            preempt_timeout: Duration = Duration(seconds=60.0),
            server_wait_timeout: Duration = Duration(seconds=60.0),
            ):
        """Constructor for SimpleActionState action client wrapper.

        @type action_name: string
        @param action_name: The name of the action as it will be broadcast over ros.

        @type action_spec: actionlib action msg
        @param action_spec: The type of action to which this client will connect.

        @type goal: actionlib goal msg
        @param goal: If the goal for this action does not need to be generated at
        runtime, it can be passed to this state on construction.

        @type goal_key: string
        @param goal_key: Pull the goal message from a key in the userdata.
        This will be done before calling the goal_cb if goal_cb is defined.

        @type goal_slots: list of string
        @param goal_slots: Pull the goal fields (__slots__) from like-named
        keys in userdata. This will be done before calling the goal_cb if
        goal_cb is defined.

        @type goal_cb: callable
        @param goal_cb: If the goal for this action needs to be generated at
        runtime, a callback can be stored which modifies the default goal
        object. The callback is passed two parameters:
            - userdata
            - current stored goal
        The callback  returns a goal message.

        @type result_key: string
        @param result_key: Put the result message into the userdata with
        the given key. This will be done after calling the result_cb
        if result_cb is defined.

        @type result_slots: list of strings
        @param result_slots: Put the result message fields (__slots__)
        into the userdata with keys like the field names. This will be done
        after calling the result_cb if result_cb is defined.

        @type result_cb: callable
        @param result_cb: If result information from this action needs to be
        stored or manipulated on reception of a result from this action, a
        callback can be stored which is passed this information. The callback
        is passed three parameters:
            - userdata (L{UserData<smach.user_data.UserData>})
            - result status (C{actionlib.GoalStatus})
            - result (actionlib result msg)

        @type exec_timeout: C{rclpy.time.Duration}
        @param exec_timeout: This is the timeout used for sending a preempt message
        to the delegate action. This is C{None} by default, which implies no
        timeout.

        @type preempt_timeout: C{rclpy.time.Duration}
        @param preempt_timeout: This is the timeout used for aborting after a
        preempt has been sent to the action and no result has been received. This
        timeout begins counting after a preempt message has been sent.

        @type server_wait_timeout: C{rclpy.time.Duration}
        @param server_wait_timeout: This is the timeout used for aborting while
        waiting for an action server to become active.
        """

        # Initialize base class
        RosState.__init__(self, node, outcomes=outcomes)

        # Set action properties
        self._action_name = action_name
        self._action_spec = action_spec

        # Set timeouts
        self._goal_status = 0
        self._goal_result = None
        self._exec_timeout = exec_timeout
        self._preempt_timeout = preempt_timeout
        self._server_wait_timeout = server_wait_timeout

        # Set goal generation policy
        if goal and hasattr(goal, '__call__'):
            raise smach.InvalidStateError("Goal object given to SimpleActionState that IS a function object")
        sl = action_spec.Goal.get_fields_and_field_types().keys()
        if not all([s in sl for s in goal_slots]):
            raise smach.InvalidStateError(f"Goal slots specified are not valid slots. Available slots: {sl}; specified slots: {goal_slots}")
        if goal_cb and not hasattr(goal_cb, '__call__'):
            raise smach.InvalidStateError("Goal callback object given to SimpleActionState that IS NOT a function object")

        # Static goal
        if goal is None:
            self._goal = action_spec.Goal()
        else:
            self._goal = goal

        # Goal from userdata key
        self._goal_key = goal_key
        if goal_key is not None:
            self.register_input_keys([goal_key])

        # Goal slots from userdata keys
        self._goal_slots = goal_slots
        self.register_input_keys(goal_slots)

        # Goal generation callback
        self._goal_cb = goal_cb
        self._goal_cb_args = goal_cb_args
        self._goal_cb_kwargs = goal_cb_kwargs
        if smach.has_smach_interface(goal_cb):
            self._goal_cb_input_keys = goal_cb.get_registered_input_keys()
            self._goal_cb_output_keys = goal_cb.get_registered_output_keys()

            self.register_input_keys(self._goal_cb_input_keys)
            self.register_output_keys(self._goal_cb_output_keys)
        else:
            self._goal_cb_input_keys = input_keys
            self._goal_cb_output_keys = output_keys

        # Set result processing policy
        if result_cb and not hasattr(result_cb, '__call__'):
            raise smach.InvalidStateError("Result callback object given to SimpleActionState that IS NOT a function object")
        if not all([s in action_spec.Result.get_fields_and_field_types().keys() for s in result_slots]):
            raise smach.InvalidStateError("Result slots specified are not valid slots.")

        # Result callback
        self._result_cb = result_cb
        self._result_cb_args = result_cb_args
        self._result_cb_kwargs = result_cb_kwargs

        if smach.has_smach_interface(result_cb):
            self._result_cb_input_keys = result_cb.get_registered_input_keys()
            self._result_cb_output_keys = result_cb.get_registered_output_keys()
            self._result_cb_outcomes = result_cb.get_registered_outcomes()

            self.register_input_keys(self._result_cb_input_keys)
            self.register_output_keys(self._result_cb_output_keys)
            self.register_outcomes(self._result_cb_outcomes)
        else:
            self._result_cb_input_keys = input_keys
            self._result_cb_output_keys = output_keys
            self._result_cb_outcomes = outcomes

        # Result to userdata key
        self._result_key = result_key
        if result_key is not None:
            self.register_output_keys([result_key])

        # Result slots to userdata keys
        self._result_slots = result_slots
        self.register_output_keys(result_slots)

        # Register additional input and output keys
        self.register_input_keys(input_keys)
        self.register_output_keys(output_keys)
        self.register_outcomes(outcomes)

        # Declare some status variables
        self._activate_time = self.node.get_clock().now()
        self._duration = Duration(seconds=0.0)
        self._status = ActionState.WAITING_FOR_SERVER

        # Construct action client and goal handle
        self._action_client = rclpy.action.ActionClient(self.node, action_spec, action_name)
        self._client_goal_handle = None

        self._execution_timer_thread = None
        # Condition variables for threading synchronization
        self._done_cond = threading.Condition()

    def _execution_timer(self):
        """Internal method for cancelling a timed out goal after a timeout."""
        self.node.get_logger().debug(
            f"Starting execution timer for action '{self._action_name}' "
            f"with timeout {self._exec_timeout.nanoseconds / 1e9} seconds."
        )
        while self._status == ActionState.ACTIVE and rclpy.ok():
            try:
                # Check for preemption while checking for timeout
                if self.preempt_requested():
                    self.node.get_logger().info(f"Preempting while waiting for goal (request) result '{self._action_name}'.")
                    self._status = ActionState.CANCELED
                    with self._done_cond:
                        self._done_cond.notify()
                if self.node.get_clock().now() - self._activate_time > self._exec_timeout:
                    self.node.get_logger().warn(
                        f"Action {self._action_name} timed out after {self._exec_timeout.nanoseconds / 1e9} seconds. "
                        f"Trying to cancel goal with timeout {self._preempt_timeout.nanoseconds / 1e9}.")
                    self._status = self._cancel_goal_and_wait(
                        timeout=self._preempt_timeout)
                    with self._done_cond:
                        self._done_cond.notify()
                time.sleep(0.1)
            except (RuntimeError, TypeError) as e:
                self.node.get_logger().error(
                    f"Caught exception: {str(e)}. Failed to sleep while running '{self._action_name}'")
                break

    # Smach State API
    def request_preempt(self):
        """Request preemption of this state and cancel the goal with a timeout."""
        self.node.get_logger().info(f"Preempt requested on action '{self._action_name}'")
        RosState.request_preempt(self)

        # If status is active, try to cancel goal with a timeout
        if self._status == ActionState.ACTIVE and rclpy.ok():
            self.node.get_logger().info(
                f"Preempt on action '{self._action_name}' trying to cancel goal: {self._client_goal_handle} "
                f"with timeout {self._preempt_timeout.nanoseconds / 1e9}")
            self._status = self._cancel_goal_and_wait(self._preempt_timeout)
            with self._done_cond:
                self._done_cond.notify()

    def _cancel_goal_and_wait(self, timeout) -> ActionState:
        """Cancel goal and wait for the result."""
        if self._client_goal_handle is not None:
            self._cancel_goal()
            self._cancel_activation_time = self.node.get_clock().now()

            while (self.node.get_clock().now() - self._cancel_activation_time) <= timeout:
                if self._goal_status == GoalStatus.STATUS_CANCELED:
                    self.node.get_logger().info(
                        f"Preempt on action '{self._action_name}'. Goal cancel succeeded within timeout .")
                    return ActionState.CANCELED
                time.sleep(0.1)

        self.node.get_logger().warn(
            f"Preempt on action '{self._action_name}' Goal cancel failed within timeout.")
        return ActionState.CANCELING

    def execute(self, ud):
        """Called when executing a state.
        This calls the goal_cb if it is defined, and then dispatches the
        goal with a non-blocking call to the action client.
        """

        # Make sure we're connected to the action server
        try:
            start_time = self.node.get_clock().now()
            while not self._action_client.server_is_ready():
                # Check for preemption while waiting for server
                if self.preempt_requested():
                    self.node.get_logger().info(f"Preempting while waiting for server '{self._action_name}'.")
                    self.service_preempt()
                    return 'preempted'
                elif not rclpy.ok():
                    self.node.get_logger().info(f"Shutting down while waiting for server '{self._action_name}'.")
                    return 'aborted'
                elif self._action_client.wait_for_server(1.0):
                    self.node.get_logger().debug(f"Connected to server '{self._action_name}'")
                elif self.node.get_clock().now() - start_time > self._server_wait_timeout:
                    self.node.get_logger().warn("Server connection timeout reached")
                    return 'aborted'
                else:
                    self.node.get_logger().warn(f"Still waiting for server '{self._action_name}'...")
        except Exception as e:
            self.node.get_logger().error(f"Caught exception: {str(e)} Failed to connect to server '{self._action_name}'. Aborting.")
            return 'aborted'

        # Server is ready, set status to inactive
        self._status = ActionState.INACTIVE

        # Grab goal key, if set
        if self._goal_key is not None:
            self._goal = ud[self._goal_key]

        # Write goal fields from userdata if set
        for key in self._goal_slots:
            setattr(self._goal, key, ud[key])

        # Call user-supplied callback, if set, to get a goal
        if self._goal_cb is not None:
            try:
                goal_update = self._goal_cb(
                        smach.Remapper(
                                ud,
                                self._goal_cb_input_keys,
                                self._goal_cb_output_keys,
                                []),
                        self._goal,
                        *self._goal_cb_args,
                        **self._goal_cb_kwargs)
                if goal_update is not None:
                    self._goal = goal_update
            except Exception as e:
                self.node.get_logger().error(
                    f"Caught exception: {str(e)} Could not execute goal callback: {traceback.format_exc()}")
                return 'aborted'

        # Make sure the necessary parameters have been set
        if self._goal is None and self._goal_cb is None:
            self.node.get_logger().error(
                f"Attempting to activate action {self._action_name} with no goal or goal callback set. "
                f"Did you construct the SimpleActionState properly?")
            return 'aborted'

        # Activate the state before sending the goal
        self._activate_time = self.node.get_clock().now()
        self._status = ActionState.ACTIVE

        with self._done_cond:
            # Dispatch goal via non-blocking call to action server
            send_future = self._action_client.send_goal_async(
                self._goal, feedback_callback=self._goal_feedback_cb)
            send_future.add_done_callback(self._goal_active_cb)

            # Execution timeout watch thread
            if self._exec_timeout:
                self._execution_timer_thread = threading.Thread(
                    name=self._action_name + '/exec_timeout_watchdog', target=self._execution_timer)
                self._execution_timer_thread.start()

            # Wait for action to finish
            self._done_cond.wait()

        # Call user result callback if defined
        result_cb_outcome = None
        if self._result_cb is not None:
            try:
                result_cb_outcome = self._result_cb(
                        smach.Remapper(
                                ud,
                                self._result_cb_input_keys,
                                self._result_cb_output_keys,
                                []),
                        self._goal_status,
                        self._goal_result)
                if result_cb_outcome is not None and result_cb_outcome not in self.get_registered_outcomes():
                    self.node.get_logger().error(
                        f"Result callback for action {self._action_name}, {self._result_cb} was not registered with "
                        f"the result_cb_outcomes argument. The result callback returned '{result_cb_outcome}' "
                        f"but the only registered outcomes are: {self.get_registered_outcomes()}")
                    return 'aborted'
            except Exception as e:
                self.node.get_logger().error(
                    f"Caught exception: {str(e)} Could not execute result callback: {traceback.format_exc()}. Aborting")
                return 'aborted'

        if self._result_key is not None:
            ud[self._result_key] = self._goal_result

        # Goal might be None, for instance if goal was LOST.
        if self._goal_result is not None:
            for key in self._result_slots:
                ud[key] = getattr(self._goal_result, key)

        # Check for preemption after executing
        if self.preempt_requested():
            self.node.get_logger().info(f"Preempting {self._action_name} after goal.")
            self.service_preempt()
            return 'preempted'

        # Check status and set the outcome to the result state
        if self._status == ActionState.COMPLETED:
            # Goal request and result are succeeded
            outcome = 'succeeded'
        elif self._status == ActionState.CANCELED:
            # Preempted or goal cancelled
            if self.preempt_requested():
                self.service_preempt()
            outcome = 'preempted'
        elif self._status == ActionState.CANCELING:
            # Preempting or exec timeout but goal not cancelled
            outcome = 'aborted'
        else:
            # All failures at this level are captured by aborting, even if we
            # timed out
            self.node.get_logger().warn("Action state terminated without going inactive first.")
            outcome = 'aborted'

        # Check custom result cb outcome
        if result_cb_outcome is not None:
            outcome = result_cb_outcome

        # Set status inactive
        self._status = ActionState.INACTIVE

        return outcome

    # Action client callbacks
    def _goal_active_cb(self, future):
        """Goal Active Callback
        Accept or reject a client request to begin an action.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().debug(f"Goal request from action {self._action_name} has been rejected!")
            return
        else:
            self.node.get_logger().debug(f"Goal request from action {self._action_name} has been accepted!")

        # Goal accepted so save goal handle
        self._client_goal_handle = goal_handle

        result_future = self._client_goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _goal_feedback_cb(self, feedback):
        """Goal Feedback Callback"""
        self.node.get_logger().debug(f"Action {self._action_name} sent feedback {feedback}")

    def _get_result_callback(self, future: Future):
        """Goal Done Callback
        This callback resets the active flags and reports the duration of the action.
        """

        def get_result_str(i):
            strs = (
                'STATUS_UNKONWN',
                'STATUS_ACCEPTED',
                'STATUS_EXECUTING',
                'STATUS_CANCELING',
                'STATUS_SUCCEEDED',
                'STATUS_CANCELED',
                'STATUS_ABORTED')
            if i < len(strs):
                return strs[i]
            else:
                return 'UNKNOWN (' + str(i) + ')'

        goal_result = future.result()

        # Calculate duration
        self._duration = self.node.get_clock().now() - self._activate_time
        self.node.get_logger().debug(f"Action {self._action_name} terminated after {self._duration} nanoseconds "
                                     f"with result {get_result_str(goal_result.status)}.")

        # Store goal state
        self._goal_status = goal_result.status
        self._goal_result = goal_result.result

        # Update status
        if self._goal_status == GoalStatus.STATUS_SUCCEEDED:
            self._status = ActionState.COMPLETED
        elif self._goal_status == GoalStatus.STATUS_CANCELED:
            self._status = ActionState.CANCELED
        elif self._goal_status == GoalStatus.STATUS_ABORTED:
            self._status = ActionState.CANCELED

        self.node.get_logger().debug(f"Goal completed: {self._client_goal_handle}")

        # Reset the goal handle
        self._client_goal_handle = None

        # Notify done
        with self._done_cond:
            self._done_cond.notify()

    def _cancel_goal(self):
        """Cancel Goal"""
        self.node.get_logger().debug("Canceling goal: " + str(self._client_goal_handle))
        if self._client_goal_handle is not None:
            self.node.get_logger().info("Canceling goal")
            future = self._client_goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_goal_callback)
        else:
            self.node.get_logger().warn("Client goal handle is None, cannot cancel goal.")

    def _cancel_goal_callback(self, future):
        """Cancel Goal Callback"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().debug("Cancel request accepted")
        else:
            self.node.get_logger().debug("Cancel request rejected")
