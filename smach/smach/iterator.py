
import threading
import traceback
from contextlib import contextmanager

import smach

__all__ = ['Iterator']

class Iterator(smach.container.Container):
    """Sequence Container

    This container inherits functionality from L{smach.StateMachine} and adds
    some auto-generated transitions that create a sequence of states from the
    order in which said states are added to the container.
    """
    def __init__(self,
            outcomes,
            input_keys,
            output_keys,
            it = [],
            it_label = 'it_data',
            exhausted_outcome = 'exhausted'):
        """Constructor.

        @type outcomes: list of string
        @param outcomes: The potential outcomes of this container.

        @type it: iterable
        @param iteritems: Items to iterate over on each cycle

        @type it_label: string
        @param iteritems_label: The label that the item in the current
        iteration will be given when it is put into the container's local
        userdata.
        """
        if exhausted_outcome not in outcomes:
            outcomes.append(exhausted_outcome)
        smach.container.Container.__init__(self, outcomes, input_keys, output_keys)

        self._items = it
        self._items_label = it_label

        self._is_running = False

        self._state_label = ''
        self._state = None
        self._loop_outcomes = []
        self._break_outcomes = []
        self._final_outcome_map = {}
        self._exhausted_outcome = exhausted_outcome


    ### Construction Methods
    @staticmethod
    def set_iteritems(it, it_label='it_data'):
        """Set the list or generator for the iterator to iterate over.
        
        @type it: iterable
        @param iteritems: Items to iterate over on each cycle

        @type it_label: string
        @param iteritems_label: The label that the item in the current
        iteration will be given when it is put into the container's local
        userdata.

        @type exhausted_outcome: string
        @param exhausted_outcome: If the iterable is exhausted without a break
        condition this outcome is emitted by the container.
        """
        # Get currently opened container
        self = Iterator._currently_opened_container()
        self._items = it
        self._items_label = it_label

    @staticmethod
    def set_contained_state(
            label,
            state,
            loop_outcomes = [],
            break_outcomes = [],
            final_outcome_map = {}):
        """Set the contained state
        
        @type label: string
        @param label: The label of the state being added.
        
        @type state: L{smach.State}
        @param state: An instance of a class implementing the L{smach.State} interface.

        @param loop_outcomes: List of contained state outcomes that should cause
        the iterator to continue. If this is empty, all outcomes that are not
        in the break_outcomes list will cause the iterator to continue to
        iterate. NOTE: loop_outcomes will be overriden by break_outcomes if both
        parameters are used.

        @param break_outcomes: List of contained state outcomes that should
        cause the iterator to break. When the contained state emits an outcome
        in this list, the container will terminate and return either that
        outcome or the outcome it is mapped to in final_outcome_map. NOTE:
        loop_outcomes will be overriden by break_outcomes if both
        parameters are used.

        @param final_outcome_map: A map from contained state outcomes to container
        outcomes. On termination of the iterator (either from finishing or from
        a break) this map will be used to translate contained state outcomes to
        container outcomes.
        Unspecified contained state outcomes will fall through as
        container outcomes.
        """
        # Get currently opened container
        self = Iterator._currently_opened_container()

        self._state_label = label
        self._state = state

        # Get potential state outcomes
        state_outcomes = state.get_registered_outcomes()

        # Check for loop and break outcomes
        if loop_outcomes and break_outcomes:
            smach.logwarn('Both loop_outcomes and break_outcomes were specified when constructing SMACH iterator container.')

        if break_outcomes:
            self._break_outcomes = break_outcomes
            for outcome in state_outcomes:
                if outcome not in break_outcomes:
                    self._loop_outcomes.append(outcome)
        else:
            self._loop_outcomes = loop_outcomes
            for outcome in state_outcomes:
                if outcome not in loop_outcomes:
                    self._break_outcomes.append(outcome)

        self._final_outcome_map = final_outcome_map

    ### State interface
    def execute(self, parent_ud):
        self._is_running = True

        # Copy input keys
        self._copy_input_keys(parent_ud, self.userdata)

        self.call_start_cbs()

        # Iterate over items
        outcome = self._exhausted_outcome

        if hasattr(self._items,'__call__'):
            it = self._items().__iter__()
        else:
            it = self._items.__iter__()

        while not smach.is_shutdown():
            try:
                item = next(it)
            except:
                outcome = self._exhausted_outcome
                break
            smach.loginfo("Iterating %s of %s" % (str(item), str(self._items)))
            self.userdata[self._items_label] = item
            # Enter the contained state
            try:
                outcome = self._state.execute(self.userdata)
            except smach.InvalidUserCodeError as ex:
                smach.logerr("Could not execute Iterator state '%s'" % self._state_label)
                raise ex
            except:
                raise smach.InvalidUserCodeError("Could not execute iterator state '%s' of type '%s': " % ( self._state_label, self._state) + traceback.format_exc())
                


            # Check if we should stop preemptively
            if self._preempt_requested\
                    or outcome in self._break_outcomes\
                    or (len(self._loop_outcomes) > 0 and outcome not in self._loop_outcomes):
                self._preempt_requested = False
                break
            self.call_transition_cbs()

        # Remap the outcome if necessary
        if outcome in self._final_outcome_map:
            outcome = self._final_outcome_map[outcome]

        # Copy output keys
        self._copy_output_keys(self.userdata, parent_ud)

        self._is_running = False

        self.call_termination_cbs(self._state_label,outcome)

        return outcome

    def request_preempt(self):
        self._preempt_requested = True
        if self._is_running:
            self._state.request_preempt()
    
    ### Container interface
    def get_children(self):
        return {self._state_label: self._state}

    def __getitem__(self,key):
        if key != self._state_label:
            smach.logerr("Attempting to get state '%s' from Iterator container. The only available state is '%s'." % (key, self._state_label))
            raise KeyError()
        return self._state

    def get_initial_states(self):
        return [self._state_label]

    def set_initial_state(self, initial_states, userdata):
        # Check initial state
        if len(initial_states) > 1:
            smach.logwarn("Attempting to set initial state to include more than one state, but Iterator container can only have one initial state." % (self._state_label))

        if len(initial_states) > 0:
            if initial_states[0] != self._state_label:
                smach.logwarn("Attempting to set state '%s' as initial state in Iterator container. The only available state is '%s'." % (initial_states[0], self._state_label))
                raise KeyError()

        # Set local userdata
        self.userdata.update(userdata)

    def get_active_states(self):
        if self._is_running:
            return [self._state_label]
        return []

    def get_internal_edges(self):
        int_edges = []
        
        for outcome in self._loop_outcomes:
            int_edges.append([outcome, self._state_label, self._state_label])

        for outcome in self._break_outcomes:
            container_outcome = outcome
            if outcome in self._final_outcome_map:
                container_outcome = self._final_outcome_map[outcome]
            if outcome == container_outcome:
                int_edges.append((outcome, self._state_label, None))
            else:
                int_edges.append((outcome, self._state_label, container_outcome))

        return int_edges

    def check_consistency(self):
        pass
