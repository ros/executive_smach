
import threading
from contextlib import contextmanager

import smach

__all__ = ['Sequence']

class Sequence(smach.state_machine.StateMachine):
    """Sequence Container

    This container inherits functionality from L{smach.StateMachine} and adds
    some auto-generated transitions that create a sequence of states from the
    order in which said states are added to the container.
    """
    def __init__(self,
            outcomes,
            connector_outcome,
            input_keys=[],
            output_keys=[]):
        """Constructor.

        @type outcomes: list of string
        @param outcomes: The potential outcomes of this container.

        @type connector_outcome: string
        @param connector_outcome: The outcome used to connect states in the
        sequence.
        """
        smach.state_machine.StateMachine.__init__(self, outcomes, input_keys, output_keys)

        self._last_added_seq_label = None
        self._connector_outcome = connector_outcome

    ### Construction Methods
    @staticmethod
    def add(label, state, transitions = None, remapping = None):
        """Add a state to the sequence.
        Each state added will receive an additional transition from it to the
        state which is added after it. The transition will follow the outcome
        specified at construction of this container.
        
        @type label: string
        @param label: The label of the state being added.
        
        @param state: An instance of a class implementing the L{State} interface.
        
        @param transitions: A dictionary mapping state outcomes to other state
        labels. If one of these transitions follows the connector outcome
        specified in the constructor, the provided transition will override
        the automatically generated connector transition.
        """
        # Get currently opened container
        self = Sequence._currently_opened_container()

        if transitions is None:
            transitions = {}

        # Perform sequence linking
        if self._last_added_seq_label is not None:
            #print self._transitions[self._last_added_seq_label]

            last_label = self._last_added_seq_label
            # Check if the connector outcome has been overriden
            if self._connector_outcome not in self._transitions[last_label]\
                    or self._transitions[last_label][self._connector_outcome] is None:
                self._transitions[last_label][self._connector_outcome] = label
            try:
                self.check_state_spec(last_label, self._states[last_label], self._transitions[last_label])
            except:
                smach.logerr("Attempting to construct smach state sequence failed.")

            #print self._transitions[self._last_added_seq_label]

        # Store the last added state label
        self._last_added_seq_label = label

        return smach.StateMachine.add(label, state, transitions, remapping)


