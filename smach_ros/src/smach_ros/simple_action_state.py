
import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback
import copy

from actionlib.simple_action_client import SimpleActionClient, GoalStatus

import smach
from smach.state import *

__all__ = ['SimpleActionState']

class SimpleActionState(State):
    """Simple action client state.
    
    Use this class to represent an actionlib as a state in a state machine.
    """

    # Meta-states for this action
    WAITING_FOR_SERVER = 0
    INACTIVE = 1
    ACTIVE = 2
    PREEMPTING = 3
    COMPLETED = 4

    def __init__(self,
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
            outcomes = [],
            # Timeouts
            exec_timeout = None,
            preempt_timeout = rospy.Duration(60.0),
            server_wait_timeout = rospy.Duration(60.0)
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

        @type exec_timeout: C{rospy.Duration}
        @param exec_timeout: This is the timeout used for sending a preempt message
        to the delegate action. This is C{None} by default, which implies no
        timeout. 

        @type preempt_timeout: C{rospy.Duration}
        @param preempt_timeout: This is the timeout used for aborting after a
        preempt has been sent to the action and no result has been received. This
        timeout begins counting after a preempt message has been sent.

        @type server_wait_timeout: C{rospy.Duration}
        @param server_wait_timeout: This is the timeout used for aborting while
        waiting for an action server to become active.
        """

        # Initialize base class
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

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
        sl = action_spec().action_goal.goal.__slots__
        if not all([s in sl for s in goal_slots]):
            raise smach.InvalidStateError("Goal slots specified are not valid slots. Available slots: %s; specified slots: %s" % (sl, goal_slots))
        if goal_cb and not hasattr(goal_cb, '__call__'):
            raise smach.InvalidStateError("Goal callback object given to SimpleActionState that IS NOT a function object")

        # Static goal
        if goal is None:
            self._goal = copy.copy(action_spec().action_goal.goal)
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
        if not all([s in action_spec().action_result.result.__slots__ for s in result_slots]):
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
        self._activate_time = rospy.Time.now()
        self._preempt_time = rospy.Time.now()
        self._duration = rospy.Duration(0.0)
        self._status = SimpleActionState.WAITING_FOR_SERVER

        # Construct action client, and wait for it to come active
        self._action_client = SimpleActionClient(action_name, action_spec)
        self._action_wait_thread = threading.Thread(name=self._action_name+'/wait_for_server', target=self._wait_for_server)
        self._action_wait_thread.start()

        self._execution_timer_thread = None

        # Condition variables for threading synchronization
        self._done_cond = threading.Condition()

    def _wait_for_server(self):
        """Internal method for waiting for the action server
        This is run in a separate thread and allows construction of this state
        to not block the construction of other states.
        """
        while self._status == SimpleActionState.WAITING_FOR_SERVER and not rospy.is_shutdown():
            try:
                if self._action_client.wait_for_server(rospy.Duration(1.0)):#self._server_wait_timeout):
                    self._status = SimpleActionState.INACTIVE
                if self.preempt_requested():
                    return
            except:
                if not rospy.core._in_shutdown: # This is a hack, wait_for_server should not throw an exception just because shutdown was called
                    rospy.logerr("Failed to wait for action server '%s'" % (self._action_name))

    def _execution_timer(self):
        """Internal method for cancelling a timed out goal after a timeout."""
        while self._status == SimpleActionState.ACTIVE and not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except:
                if not rospy.is_shutdown():
                    rospy.logerr("Failed to sleep while running '%s'" % self._action_name)
            if rospy.Time.now() - self._activate_time > self._exec_timeout:
                rospy.logwarn("Action %s timed out after %d seconds." % (self._action_name, self._exec_timeout.to_sec()))
                # Cancel the goal
                self._action_client.cancel_goal()

    ### smach State API
    def request_preempt(self):
        rospy.loginfo("Preempt requested on action '%s'" % (self._action_name))
        smach.State.request_preempt(self)
        if self._status == SimpleActionState.ACTIVE:
            rospy.loginfo("Preempt on action '%s' cancelling goal: \n%s" % (self._action_name, str(self._goal)))
            # Cancel the goal
            self._action_client.cancel_goal()

    def execute(self, ud):
        """Called when executing a state.
        This calls the goal_cb if it is defined, and then dispatches the
        goal with a non-blocking call to the action client.
        """

        # Make sure we're connected to the action server
        if self._status is SimpleActionState.WAITING_FOR_SERVER:
            rospy.logwarn("Still waiting for action server '%s' to start... is it running?" % self._action_name)
            if self._action_wait_thread.is_alive():
                # Block on joining the server wait thread (This can be preempted)
                self._action_wait_thread.join()
            else:
                # Wait for the server in this thread (This can also be preempted)
                self._wait_for_server()

            if not self.preempt_requested():
                # In case of preemption we probably didn't connect
                rospy.loginfo("Connected to action server '%s'." % self._action_name)

        # Check for preemption before executing
        if self.preempt_requested():
            rospy.loginfo("Preempting %s before sending goal." % self._action_name)
            self.service_preempt()
            return 'preempted'

        # We should only get here if we have connected to the server
        if self._status is SimpleActionState.WAITING_FOR_SERVER:
            rospy.logfatal("Action server for "+self._action_name+" is not running.")
            return 'aborted'
        else:
            self._status = SimpleActionState.INACTIVE

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
            except:
                rospy.logerr("Could not execute goal callback: "+traceback.format_exc())
                return 'aborted'
            
        # Make sure the necessary paramters have been set
        if self._goal is None and self._goal_cb is None:
            rospy.logerr("Attempting to activate action "+self._action_name+" with no goal or goal callback set. Did you construct the SimpleActionState properly?")
            return 'aborted'

        # Dispatch goal via non-blocking call to action client
        self._activate_time = rospy.Time.now()
        self._status = SimpleActionState.ACTIVE

        # Wait on done condition
        self._done_cond.acquire()
        self._action_client.send_goal(self._goal, self._goal_done_cb, self._goal_active_cb, self._goal_feedback_cb)

        # Preempt timeout watch thread
        if self._exec_timeout:
            self._execution_timer_thread = threading.Thread(name=self._action_name+'/preempt_watchdog', target=self._execution_timer)
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
                    rospy.logerr("Result callback for action "+self._action_name+", "+str(self._result_cb)+" was not registered with the result_cb_outcomes argument. The result callback returned '"+str(result_cb_outcome)+"' but the only registered outcomes are: "+str(self.get_registered_outcomes()))
                    return 'aborted'
            except:
                rospy.logerr("Could not execute result callback: "+traceback.format_exc())
                return 'aborted'

        if self._result_key is not None:
            ud[self._result_key] = self._goal_result

        for key in self._result_slots:
            ud[key] = getattr(self._goal_result, key)

        # Check status
        if self._status == SimpleActionState.INACTIVE:
            # Set the outcome on the result state
            if self._goal_status == GoalStatus.SUCCEEDED:
                outcome = 'succeeded'
            elif self._goal_status == GoalStatus.PREEMPTED and self.preempt_requested():
                outcome = 'preempted'
                self.service_preempt()
            else:
                # All failures at this level are captured by aborting, even if we timed out
                # This is an important distinction between local preemption, and preemption
                # from above.
                outcome = 'aborted'
        else:
            # We terminated without going inactive
            rospy.logwarn("Action state terminated without going inactive first.")
            outcome = 'aborted'

        # Check custom result cb outcome
        if result_cb_outcome is not None:
            outcome = result_cb_outcome

        # Set status inactive
        self._status = SimpleActionState.INACTIVE
        self._done_cond.release()

        return outcome

    ### Action client callbacks
    def _goal_active_cb(self):
        """Goal Active Callback
        This callback starts the timer that watches for the timeout specified for this action.
        """
        rospy.logdebug("Action "+self._action_name+" has gone active.")

    def _goal_feedback_cb(self, feedback):
        """Goal Feedback Callback"""
        rospy.logdebug("Action "+self._action_name+" sent feedback.")

    def _goal_done_cb(self, result_state, result):
        """Goal Done Callback
        This callback resets the active flags and reports the duration of the action.
        Also, if the user has defined a result_cb, it is called here before the
        method returns.
        """
        def get_result_str(i):
            strs = ('PENDING','ACTIVE','PREEMPTED','SUCCEEDED','ABORTED','REJECTED','LOST')
            if i < len(strs):
                return strs[i]
            else:
                return 'UNKNOWN ('+str(i)+')'

        # Calculate duration
        self._duration = rospy.Time.now() - self._activate_time
        rospy.logdebug("Action "+self._action_name+" terminated after "\
                +str(self._duration.to_sec())+" seconds with result "\
                +get_result_str(self._action_client.get_state())+".")

        # Store goal state
        self._goal_status = result_state
        self._goal_result = result

        # Rest status
        self._status = SimpleActionState.INACTIVE

        # Notify done
        self._done_cond.acquire()
        self._done_cond.notify()
        self._done_cond.release()
