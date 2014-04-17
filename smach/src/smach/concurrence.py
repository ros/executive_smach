import threading
import traceback
import copy
from contextlib import contextmanager

import smach

__all__ = ['Concurrence']

class Concurrence(smach.container.Container):
    """Concurrence Container

    This state allows for simple split-join concurrency. The user adds a set of
    states which are all executed simultaneously. The concurrent split state
    can only transition once all conatained states are ready to transition.
    
    This container can be configured to return a given outcome as a function of
    the outcomes of the contained states. This is specified in the constructor
    of the class, or after construction with L{Concurrence.add_outcome_map}.

    While a concurrence will not terminate until all if its children terminate,
    it is possible for it to preempt a subset of states 
     - All child states terminate
     - At least one child state terminates
     - A user-defined callback signals termination

    Given these causes of termination, the outcome can be determined in four ways:
     - A user-defined callback returns an outcome
     - A child-outcome map which requires ALL states to terminate is satisfied
     - A child-outcome map which requires ONE state to terminate is satisfied
     - No  maps are satisfied, so the default outcome is returned

    The specification of the outcome maps and the outcome callback are
    described in the constructor documentation below. More than one policy can
    be supplied, and each policy has the potential to not be satisfied. In the
    situation in which multiple policies are provided, and a given policy is
    not satisfied, the outcome choice precedence is as follows:
     - Outcome callback
     - First-triggered outcome map
     - last-triggered outcome map
     - Default outcome

    In practive it is best to try to accomplish your task with just ONE outcome
    policy.

    """
    def __init__(self,
            outcomes,
            default_outcome,
            input_keys = [],
            output_keys = [],
            outcome_map = {},
            outcome_cb = None,
            child_termination_cb = None
            ):
        """Constructor for smach Concurrent Split.

        @type outcomes: list of strings
        @param outcomes: The potential outcomes of this state machine.

        @type default_outcome: string
        @param default_outcome: The outcome of this state if no elements in the 
        outcome map are satisfied by the outcomes of the contained states.


        @type outcome_map: list
        @param outcome_map: This is an outcome map for determining the
        outcome of this container. Each outcome of the container is mapped
        to a dictionary mapping child labels onto outcomes. If none of the
        child-outcome maps is satisfied, the concurrence will terminate
        with thhe default outcome.
        
        For example, if the and_outcome_map is:
            {'succeeded' : {'FOO':'succeeded', 'BAR':'done'},
             'aborted' : {'FOO':'aborted'}}
        Then the concurrence will terimate with outcome 'succeeded' only if
        BOTH states 'FOO' and 'BAR' have terminated
        with outcomes 'succeeded' and 'done', respectively. The outcome
        'aborted' will be returned by the concurrence if the state 'FOO'
        returns the outcome 'aborted'. 

        If the outcome of a state is not specified, it will be treated as
        irrelevant to the outcome of the concurrence

        If the criteria for one outcome is the subset of another outcome,
        the container will choose the outcome which has more child outcome
        criteria satisfied. If both container outcomes have the same
        number of satisfied criteria, the behavior is undefined.

        If a more complex outcome policy is required, see the user can
        provide an outcome callback. See outcome_cb, below.

        @type child_termination_cb: callale
        @param child_termination_cb: This callback gives the user the ability
        to force the concurrence to preempt running states given the
        termination of some other set of states. This is useful when using
        a concurrence as a monitor container. 

        This callback is called each time a child state terminates. It is
        passed a single argument, a dictionary mapping child state labels
        onto their outcomes. If a state has not yet terminated, it's dict
        value will be None.

        This function can return three things:
         - False: continue blocking on the termination of all other states
         - True: Preempt all other states
         - list of state labels: Preempt only the specified states

        I{If you just want the first termination to cause the other children
        to terminate, the callback (lamda so: True) will always return True.}

        @type outcome_cb: callable
        @param outcome_cb: If the outcome policy needs to be more complicated
        than just a conjunction of state outcomes, the user can supply
        a callback for specifying the outcome of the container.

        This callback is called only once all child states have terminated,
        and it is passed the dictionary mapping state labels onto their
        respective outcomes.

        If the callback returns a string, it will treated as the outcome of
        the container.

        If the callback returns None, the concurrence will first check the
        outcome_map, and if no outcome in the outcome_map is satisfied, it
        will return the default outcome.

        B{NOTE: This callback should be a function ONLY of the outcomes of
        the child states. It should not access any other resources.} 

        """
        smach.container.Container.__init__(self, outcomes, input_keys, output_keys)

        # List of concurrent states
        self._states = {}
        self._threads = {}
        self._remappings = {}

        if not (default_outcome or outcome_map or outcome_cb):
            raise smach.InvalidStateError("Concurrence requires an outcome policy")

        # Initialize error string
        errors = ""

        # Check if default outcome is necessary
        if default_outcome != str(default_outcome):
            errors += "\n\tDefault outcome '%s' does not appear to be a string." % str(default_outcome)
        if default_outcome not in outcomes:
            errors += "\n\tDefault outcome '%s' is unregistered." % str(default_outcome)

        # Check if outcome maps only contain outcomes that are registered
        for o in outcome_map:
            if o not in outcomes:
                errors += "\n\tUnregistered outcome '%s' in and_outcome_map." % str(o)

        # Check if outcome cb is callable
        if outcome_cb and not hasattr(outcome_cb,'__call__'):
            errors += "\n\tOutcome callback '%s' is not callable." % str(outcome_cb)

        # Check if child termination cb is callable
        if child_termination_cb and not hasattr(child_termination_cb,'__call__'):
            errors += "\n\tChild termination callback '%s' is not callable." % str(child_termination_cb)

        # Report errors
        if len(errors) > 0:
            raise smach.InvalidStateError("Errors specifying outcome policy of concurrence: %s" % errors)

        # Store outcome policies
        self._default_outcome = default_outcome
        self._outcome_map = outcome_map
        self._outcome_cb = outcome_cb
        self._child_termination_cb = child_termination_cb
        self._child_outcomes = {}

        # Condition variables for threading synchronization
        self._user_code_exception = False
        self._done_cond = threading.Condition()
        self._ready_event =  threading.Event()

    ### Construction methods
    @staticmethod
    def add(label, state, remapping={}):
        """Add state to the opened concurrence.
        This state will need to terminate before the concurrence terminates.
        """
        # Get currently opened container
        self = Concurrence._currently_opened_container()

        # Store state
        self._states[label] = state
        self._remappings[label] = remapping

        return state

    ### State interface
    def execute(self, parent_ud = smach.UserData()):
        """Overridden execute method.
        This starts all the threads.
        """
        # Clear the ready event
        self._ready_event.clear()
        
        # Reset child outcomes
        self._child_outcomes = {}

        # Copy input keys
        self._copy_input_keys(parent_ud, self.userdata)

        # Spew some info
        smach.loginfo("Concurrence starting with userdata: \n\t%s" %
                (str(list(self.userdata.keys()))))

        # Call start callbacks
        self.call_start_cbs()

        # Create all the threads
        for (label, state) in ((k,self._states[k]) for k in self._states):
            # Initialize child outcomes
            self._child_outcomes[label] = None
            self._threads[label] = threading.Thread(
                    name='concurrent_split:'+label,
                    target=self._state_runner,
                    args=(label,))

        # Launch threads
        for thread in self._threads.values():
            thread.start()
        
        # Wait for done notification
        self._done_cond.acquire()
        
        # Notify all threads ready to go
        self._ready_event.set()
        
        # Wait for a done notification from a thread
        self._done_cond.wait()
        self._done_cond.release()

        # Preempt any running states
        smach.logdebug("SMACH Concurrence preempting running states.")
        for label in self._states:
            if self._child_outcomes[label] == None:
                self._states[label].request_preempt()

        # Wait for all states to terminate
        while not smach.is_shutdown():
            if all([not t.isAlive() for t in self._threads.values()]):
                break
            self._done_cond.acquire()
            self._done_cond.wait(0.1)
            self._done_cond.release()

        # Check for user code exception
        if self._user_code_exception:
            self._user_code_exception = False
            raise smach.InvalidStateError("A concurrent state raised an exception during execution.")

        # Check for preempt
        if self.preempt_requested():
            # initialized serviced flag
            children_preempts_serviced = True

            # Service this preempt if 
            for (label,state) in ((k,self._states[k]) for k in self._states):
                if state.preempt_requested():
                    # Reset the flag
                    children_preempts_serviced = False
                    # Complain
                    smach.logwarn("State '%s' in concurrence did not service preempt." % label) 
                    # Recall the preempt if it hasn't been serviced
                    state.recall_preempt()
            if children_preempts_serviced:
                smach.loginfo("Concurrence serviced preempt.")
                self.service_preempt()

        # Spew some debyg info
        smach.loginfo("Concurrent Outcomes: "+str(self._child_outcomes))

        # Initialize the outcome
        outcome = self._default_outcome

        # Determine the outcome from the outcome map
        smach.logdebug("SMACH Concurrence determining contained state outcomes.")
        for (container_outcome, outcomes) in ((k,self._outcome_map[k]) for k in self._outcome_map):
            if all([self._child_outcomes[label] == outcomes[label] for label in outcomes]):
                smach.logdebug("Terminating concurrent split with mapped outcome.")
                outcome = container_outcome

        # Check outcome callback
        if self._outcome_cb:
            try:
                cb_outcome = self._outcome_cb(copy.copy(self._child_outcomes))
                if cb_outcome:
                    if cb_outcome == str(cb_outcome):
                        outcome = cb_outcome
                    else:
                        smach.logerr("Outcome callback returned a non-string '%s', using default outcome '%s'" % (str(cb_outcome), self._default_outcome))
                else:
                    smach.logwarn("Outcome callback returned None, using outcome '%s'" % outcome)
            except:
                raise smach.InvalidUserCodeError(("Could not execute outcome callback '%s': " % self._outcome_cb)+traceback.format_exc())

        # Cleanup
        self._threads = {}
        self._child_outcomes = {}

        # Call termination callbacks
        self.call_termination_cbs(list(self._states.keys()), outcome)

        # Copy output keys
        self._copy_output_keys(self.userdata, parent_ud)

        return outcome

    def request_preempt(self):
        """Preempt all contained states."""
        # Set preempt flag
        smach.State.request_preempt(self)

        # Notify concurrence that it should preempt running states and terminate
        with self._done_cond:
            self._done_cond.notify_all()


    def _state_runner(self,label):
        """Runs the states in parallel threads."""

        # Wait until all threads are ready to start before beginnging
        self._ready_event.wait()
        
        self.call_transition_cbs()

        # Execute child state
        try:
            self._child_outcomes[label] = self._states[label].execute(smach.Remapper(
                self.userdata,
                self._states[label].get_registered_input_keys(),
                self._states[label].get_registered_output_keys(),
                self._remappings[label]))
        except:
            self._user_code_exception = True
            with self._done_cond:
                self._done_cond.notify_all()
            raise smach.InvalidStateError(("Could not execute child state '%s': " % label)+traceback.format_exc())

        # Make sure the child returned an outcome
        if self._child_outcomes[label] is None:
            raise smach.InvalidStateError("Concurrent state '%s' returned no outcome on termination." % label)
        else:
            smach.loginfo("Concurrent state '%s' returned outcome '%s' on termination." % (label, self._child_outcomes[label]))

        # Check if all of the states have completed
        with self._done_cond:
            # initialize preemption flag
            preempt_others = False
            # Call transition cb's
            self.call_transition_cbs()
            # Call child termination cb if it's defined
            if self._child_termination_cb:
                try:
                    preempt_others = self._child_termination_cb(self._child_outcomes)
                except:
                    raise smach.InvalidUserCodeError("Could not execute child termination callback: "+traceback.format_exc())

            # Notify the container to terminate (and preempt other states if neceesary)
            if preempt_others or all([o is not None for o in self._child_outcomes.values()]):
                self._done_cond.notify_all()

    ### Container interface
    def get_children(self):
        return self._states

    def __getitem__(self,key):
        return self._states[key]

    def get_initial_states(self):
        return list(self._states.keys())

    def set_initial_state(self, initial_states, userdata):
        if initial_states > 0:
            if initial_states < len(self._states):
                smach.logwarn("Attempting to set initial states in Concurrence"
                              " container, but Concurrence children are always"
                              " all executed initially, ignoring call.")

        # Set local userdata
        self.userdata.update(userdata)

    def get_active_states(self):
        return [label for (label,outcome) in ((k,self._child_outcomes[k]) for k in self._child_outcomes) if outcome is None]

    def get_internal_edges(self):
        int_edges = []
        for (container_outcome, outcomes) in ((k,self._outcome_map[k]) for k in self._outcome_map):
            for state_key in outcomes:
                int_edges.append((outcomes[state_key], state_key, container_outcome))
        return int_edges

    def check_consistency(self):
        for (co,cso) in ((k,self._outcome_map[k]) for k in self._outcome_map):
            for state_label,outcome in ((k,cso[k]) for k in cso):
                if outcome not in self._states[state_label].get_registered_outcomes():
                    raise smach.InvalidTransitionError(
                            'Outcome map in SMACH Concurrence references a state outcome that does not exist. Requested state outcome: \'%s\', but state \'%s\' only has outcomes %s' %
                            (outcome, state_label, str(self._states[state_label].get_registered_outcomes())))

