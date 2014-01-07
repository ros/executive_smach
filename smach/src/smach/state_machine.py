
import threading
import traceback
from contextlib import contextmanager

import smach

__all__ = ['StateMachine']

### State Machine class
class StateMachine(smach.container.Container):
    """StateMachine
    
    This is a finite state machine smach container. Note that though this is
    a state machine, it also implements the L{smach.State}
    interface, so these can be composed hierarchically, if such a pattern is
    desired.

    States are added to the state machine as 3-tuple specifications:
     - label
     - state instance
     - transitions

    The label is a string, the state instance is any class that implements the
    L{smach.State} interface, and transitions is a dictionary mapping strings onto
    strings which represent the transitions out of this new state. Transitions
    can take one of three forms:
     - OUTCOME -> STATE_LABEL
     - OUTCOME -> None (or unspecified)
     - OUTCOME -> SM_OUTCOME
    """
    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        """Constructor for smach StateMachine Container.

        @type outcomes: list of strings
        @param outcomes: The potential outcomes of this state machine.
        """

        # Call super's constructor
        smach.container.Container.__init__(self, outcomes, input_keys, output_keys)
        
        # Properties
        self._state_transitioning_lock = threading.Lock()

        # Current state of the state machine
        self._is_running = False # True when a goal has been dispatched to and accepted by the state machine

        self._initial_state_label = None

        self._current_label = None
        self._current_state = None
        self._current_transitions = None
        self._current_outcome = None

        # The label of the last preempted state
        self._preempted_label = None
        self._preempted_state = None

        # State machine storage
        # These are dictionaries of State objects and transition dictionaries
        # keyed on unique labels
        self._states = {}
        self._transitions = {}
        self._remappings = {}

        # Construction vars
        self._last_added_label = None
        self._connector_outcomes = []

        # Thread for execution of state switching
        self._execute_thread = None
        self.userdata = smach.UserData()

    ### Construction methods
    @staticmethod
    def add(label, state, transitions=None, remapping=None):
        """Add a state to the opened state machine.
        
        @type label: string
        @param label: The label of the state being added.
        
        @param state: An instance of a class implementing the L{State} interface.
        
        @param transitions: A dictionary mapping state outcomes to other state
        labels or container outcomes.

        @param remapping: A dictrionary mapping local userdata keys to userdata
        keys in the container.
        """
        # Get currently opened container
        self = StateMachine._currently_opened_container()

        smach.logdebug('Adding state (%s, %s, %s)' % (label, str(state), str(transitions)))

        # Set initial state if it is still unset
        if self._initial_state_label is None:
            self._initial_state_label = label

        if transitions is None:
            transitions = {}

        if remapping is None:
            remapping = {}

        # Add group transitions to this new state, if they exist
        """
        if 'transitions' in smach.Container._context_kwargs:
            for outcome, target in smach.Container._context_kwargs['transitions'].iteritems():
                if outcome not in transitions:
                    transitions[outcome] = target
        """

        # Check the state specification
        self.check_state_spec(label, state, transitions)

        # Check if th label already exists
        if label in self._states:
            raise smach.InvalidStateError(
            'Attempting to add state with label "'+label+'" to state machine, but this label is already being used.')

        # Debug info
        smach.logdebug("Adding state '"+str(label)+"' to the state machine.")

        # Create implicit terminal transitions, and combine them with the explicit transitions
        registered_outcomes = state.get_registered_outcomes()

        # Get a list of the unbound transitions
        missing_transitions = {o: None for o in registered_outcomes if o not in transitions}
        transitions.update(missing_transitions)
        smach.logdebug("State '%s' is missing transitions: %s" % (label, str(missing_transitions)))

        # Add state and transitions to the dictionary
        self._states[label] = state
        self._transitions[label] = transitions
        self._remappings[label] = remapping
        smach.logdebug("TRANSITIONS FOR %s: %s" % (label, str(self._transitions[label])))

        # Add transition to this state if connected outcome is defined
        if len(self._connector_outcomes) > 0 and self._last_added_label is not None:
            for connector_outcome in self._connector_outcomes:
                self._transitions[self._last_added_label][connector_outcome] = label
            # Reset connector outcomes and last added label
            self._connector_outcomes = []
            self._last_added_label = None

        return state

    @staticmethod
    def add_auto(label, state, connector_outcomes, transitions=None, remapping=None):
        """Add a state to the state machine such that it automatically
        transitions to the next added state.

        Each state added will receive an additional transition from it
        to the state which is added after it for every outcome given
        via connector_outcomes.

        @type label: string
        @param label: The label of the state being added.

        @param state: An instance of a class implementing the L{State} interface.

        @type connector_outcomes: list of string
        @param connector_outcomes: For which of the added state's outcomes a
        transition to the next added state should be generated.

        @param transitions: A dictionary mapping state outcomes to other state
        labels. If one of these transitions follows the connector outcome
        specified in the constructor, the provided transition will override
        the automatically generated connector transition.
        """
        # Get currently opened container
        self = StateMachine._currently_opened_container()

        # First add this state
        add_ret = smach.StateMachine.add(label, state, transitions, remapping)

        # Make sure the connector outcomes are valid for this state
        registered_outcomes = state.get_registered_outcomes()
        if not all(co in registered_outcomes for co in connector_outcomes):
            raise smach.InvalidStateError("Attempting to auto-connect states with outcomes %s, but state '%s' only has registerd outcomes: %s" % (str(connector_outcomes), str(label), str(registered_outcomes)))

        # Store this state as the last state and store the connector outcomes
        self._last_added_label = label
        self._connector_outcomes = connector_outcomes

        return add_ret

    ### Internals
    def _set_current_state(self, state_label):
        if state_label is not None:
            # Store the current label and states 
            self._current_label = state_label
            self._current_state = self._states[state_label]
            self._current_transitions = self._transitions[state_label]
            self._current_outcome = None
        else:
            # Store the current label and states 
            self._current_label = None
            self._current_state = None
            self._current_transitions = None
            self._current_outcome = None

    def _update_once(self):
        """Method that updates the state machine once.
        This checks if the current state is ready to transition, if so, it
        requests the outcome of the current state, and then extracts the next state
        label from the current state's transition dictionary, and then transitions
        to the next state.
        """
        outcome = None
        transition_target = None
        last_state_label = self._current_label

        # Make sure the state exists
        if self._current_label not in self._states:
            raise smach.InvalidStateError("State '%s' does not exist. Available states are: %s" %
                    (self._current_label, list(self._states.keys())))

        # Check if a preempt was requested before or while the last state was running
        if self.preempt_requested():
            smach.loginfo("Preempt requested on state machine before executing the next state.")
            # We were preempted
            if self._preempted_state is not None:
                # We were preempted while the last state was running
                if self._preempted_state.preempt_requested():
                    smach.loginfo("Last state '%s' did not service preempt. Preempting next state '%s' before executing..." % (self._preempted_label, self._current_label))
                    # The flag was not reset, so we need to keep preempting 
                    # (this will reset the current preempt)
                    self._preempt_current_state()
                else:
                    # The flag was reset, so the container can reset
                    self._preempt_requested = False
                    self._preempted_state = None
            else:
                # We were preempted after the last state was running
                # So we should preempt this state before we execute it
                self._preempt_current_state()

        # Execute the state
        try:
            self._state_transitioning_lock.release()
            outcome = self._current_state.execute(
                    smach.Remapper(
                        self.userdata,
                        self._current_state.get_registered_input_keys(),
                        self._current_state.get_registered_output_keys(),
                        self._remappings[self._current_label]))
        except smach.InvalidUserCodeError as ex:
            smach.logerr("State '%s' failed to execute." % self._current_label)
            raise ex
        except:
            raise smach.InvalidUserCodeError("Could not execute state '%s' of type '%s': " %
                                             (self._current_label, self._current_state)
                                             + traceback.format_exc())
        finally:
            self._state_transitioning_lock.acquire()

        # Check if outcome was a potential outcome for this type of state
        if outcome not in self._current_state.get_registered_outcomes():
            raise smach.InvalidTransitionError(
                    "Attempted to return outcome '%s' from state '%s' of"
                    " type '%s' which only has registered outcomes: %s" %
                    (outcome,
                     self._current_label,
                     self._current_state,
                     self._current_state.get_registered_outcomes()))

        # Check if this outcome is actually mapped to any target
        if outcome not in self._current_transitions:
            raise smach.InvalidTransitionError("Outcome '%s' of state '%s' is not bound to any transition target. Bound transitions include: %s" %
                    (str(outcome), str(self._current_label), str(self._current_transitions)))
        
        # Set the transition target
        transition_target = self._current_transitions[outcome]

        # Check if the transition target is a state in this state machine, or an outcome of this state machine
        if transition_target in self._states:
            # Set the new state 
            self._set_current_state(transition_target)

            # Spew some info
            smach.loginfo("State machine transitioning '%s':'%s'-->'%s'" %
                          (last_state_label, outcome, transition_target))

            # Call transition callbacks
            self.call_transition_cbs()
        else:
            # This is a terminal state
            
            if self._preempt_requested and self._preempted_state is not None:
                if not self._current_state.preempt_requested():
                    self.service_preempt()

            if transition_target not in self.get_registered_outcomes():
                # This is a container outcome that will fall through
                transition_target = outcome

            if transition_target in self.get_registered_outcomes():
                # The transition target is an outcome of the state machine
                self._set_current_state(None)

                # Spew some info
                smach.loginfo("State machine terminating '%s':'%s':'%s'" %
                              (last_state_label, outcome, transition_target))

                # Call termination callbacks
                self.call_termination_cbs([last_state_label],transition_target)

                return transition_target
            else:
                raise smach.InvalidTransitionError("Outcome '%s' of state '%s' with transition target '%s' is neither a registered state nor a registered container outcome." %
                        (outcome, self._current_label, transition_target))
        return None

    ### State Interface
    def execute(self, parent_ud = smach.UserData()):
        """Run the state machine on entry to this state.
        This will set the "closed" flag and spin up the execute thread. Once
        this flag has been set, it will prevent more states from being added to
        the state machine. 
        """

        # This will prevent preempts from getting propagated to non-existent children
        with self._state_transitioning_lock:
            # Check state consistency
            try:
                self.check_consistency()
            except (smach.InvalidStateError, smach.InvalidTransitionError):
                smach.logerr("Container consistency check failed.")
                return None

            # Set running flag
            self._is_running = True

            # Initialize preempt state
            self._preempted_label = None
            self._preempted_state = None

            # Set initial state 
            self._set_current_state(self._initial_state_label)

            # Copy input keys
            self._copy_input_keys(parent_ud, self.userdata)

            # Spew some info
            smach.loginfo("State machine starting in initial state '%s' with userdata: \n\t%s" %
                    (self._current_label, list(self.userdata.keys())))


            # Call start callbacks
            self.call_start_cbs()

            # Initialize container outcome
            container_outcome = None

            # Step through state machine
            while container_outcome is None and self._is_running and not smach.is_shutdown():
                # Update the state machine
                container_outcome = self._update_once()

            # Copy output keys
            self._copy_output_keys(self.userdata, parent_ud)

            # We're no longer running
            self._is_running = False

        return container_outcome

    ## Preemption management
    def request_preempt(self):
        """Propagate preempt to currently active state.
        
        This will attempt to preempt the currently active state.
        """
        with self._state_transitioning_lock:
            # Aleways Set this container's preempted flag
            self._preempt_requested = True
            # Only propagate preempt if the current state is defined
            if self._current_state is not None:
                self._preempt_current_state()

    def _preempt_current_state(self):
        """Preempt the current state (might not be executing yet).
        This also resets the preempt flag on a state that had previously received the preempt, but not serviced it."""
        if self._preempted_state != self._current_state:
            if self._preempted_state is not None:
                # Reset the previously preempted state (that has now terminated)
                self._preempted_state.recall_preempt()

            # Store the label of the currently active state
            self._preempted_state = self._current_state
            self._preempted_label = self._current_label

            # Request the currently active state to preempt
            try:
                self._preempted_state.request_preempt()
            except:
                smach.logerr("Failed to preempt contained state '%s': %s" % (self._preempted_label, traceback.format_exc()))

    ### Container interface
    def get_children(self):
        return self._states

    def __getitem__(self,key):
        if key not in self._states:
            smach.logerr("Attempting to get state '%s' from StateMachine container. The only available states are: %s" % (key, str(list(self._states.keys()))))
            raise KeyError()
        return self._states[key]

    def set_initial_state(self, initial_states, userdata=smach.UserData()):
        smach.logdebug("Setting initial states to " + str(initial_states))

        if len(initial_states) > 1:
            smach.logwarn("Attempting to set initial state to include more than"
                          " one state, but the StateMachine container can only"
                          " have one initial state. Taking the first one.")

        # Set the initial state label
        if len(initial_states) > 0:
            self._initial_state_label = initial_states[0]
        # Set local userdata
        self.userdata.update(userdata)

    def get_active_states(self):
        return [str(self._current_label)]

    def get_initial_states(self):
        return [str(self._initial_state_label)]

    def get_internal_edges(self):
        int_edges = []
        for (from_label,transitions) in ((k,self._transitions[k]) for k in self._transitions):
            for (outcome,to_label) in ((k,transitions[k]) for k in transitions):
                int_edges.append((outcome, from_label, to_label))
        return int_edges

    ### Validation methods
    def check_state_spec(self, label, state, transitions):
        """Validate full state specification (label, state, and transitions).
        This checks to make sure the required variables are in the state spec,
        as well as verifies that all outcomes referenced in the transitions
        are registered as valid outcomes in the state object. If a state
        specification fails validation, a L{smach.InvalidStateError} is
        thrown.
        """
        # Make sure all transitions are from registered outcomes of this state
        registered_outcomes = state.get_registered_outcomes()
        for outcome in transitions:
            if outcome not in registered_outcomes:
                raise smach.InvalidTransitionError("Specified outcome '"+outcome+"' on state '"+label+"', which only has available registered outcomes: "+str(registered_outcomes))

    def check_consistency(self):
        """Check the entire state machine for consistency.
        This asserts that all transition targets are states that are in the
        state machine. If this fails, it raises an L{InvalidTransitionError}
        with relevant information.
        """
        # Construct a set of available states
        available_states = set(list(self._states.keys())+list(self.get_registered_outcomes()))

        # Grab the registered outcomes for the state machine
        registered_sm_outcomes = self.get_registered_outcomes()

        # Hopefully this string stays empty
        errors = ""

        # Check initial_state_label
        if self._initial_state_label is None:
            errors = errors + "\n\tNo initial state set."
        elif self._initial_state_label not in self._states:
            errors = errors + "\n\tInitial state label: '"+str(self._initial_state_label)+"' is not in the state machine."

        # Generate state specifications
        state_specs = [(label, self._states[label], self._transitions[label])
                       for label in self._states]
        # Iterate over all states
        for label,state,transitions in state_specs:
            # Check that all potential outcomes are registered in this state
            transition_states = set([s for s in transitions.values()
                                     if s is not None and s != ''])
            # Generate a list of missing states
            missing_states = transition_states.difference(available_states)

            # Check number of missing states
            if len(missing_states) > 0:
                errors = (errors
                          + "\n\tState '" + str(label)
                          + "' references unknown states: " + str(list(missing_states)))

            # Check terminal outcomes for this state
            terminal_outcomes = set([o for (o, s) in ((k, transitions[k])
                                                      for k in transitions)
                                     if s is None or s == ''])
            # Terminal outcomes should be in the registered outcomes of this state machine
            missing_outcomes = terminal_outcomes.difference(registered_sm_outcomes)
            # Check number of missing outcomes
            if len(missing_outcomes) > 0:
                errors = (errors
                          + "\n\tState '" + str(label)
                          + "' references unregistered outcomes: " + str(list(missing_outcomes)))

        # Check errors
        if len(errors) > 0:
            raise smach.InvalidTransitionError("State machine failed consistency check: "+errors+"\n\n\tAvailable states: "+str(list(available_states)))

    ### Introspection methods
    def is_running(self):
        """Returns true if the state machine is running."""
        return self._is_running
