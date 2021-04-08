
import threading
import traceback

import smach

__all__ = ['State','CBState']

class State(object):
    """Base class for SMACH states.

    A SMACH state interacts with SMACH containers in two ways. The first is its
    outcome identifier, and the second is the set of userdata variables which
    it reads from and writes to at runtime. Both of these interactions are
    declared before the state goes active (when its C{execute()} method is
    called) and are checked during construction.
    """
    def __init__(self, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        """State constructor
        @type outcomes: list of str
        @param outcomes: Custom outcomes for this state.

        @type input_keys: list of str
        @param input_keys: The userdata keys from which this state might read
        at runtime. 

        @type output_keys: list of str
        @param output_keys: The userdata keys to which this state might write
        at runtime.

        @type io_keys: list of str
        @param io_keys: The userdata keys to which this state might write or
        from which it might read at runtime.
        """
        # Store outcomes
        self._outcomes = set(outcomes)

        # Store userdata interface description
        self._input_keys = set(input_keys + io_keys)
        self._output_keys = set(output_keys + io_keys)

        # Declare preempt flag
        self._preempt_requested = False

    ### Meat
    def execute(self, ud):
        """Called when executing a state.
        In the base class this raises a NotImplementedError.

        @type ud: L{UserData} structure
        @param ud: Userdata for the scope in which this state is executing
        """
        raise NotImplementedError()
    
    ### SMACH Interface API
    def register_outcomes(self, new_outcomes):
        """Add outcomes to the outcome set."""
        self._outcomes = self._outcomes.union(new_outcomes)

    def get_registered_outcomes(self):
        """Get a list of registered outcomes.
        @rtype: tuple of str
        @return: Tuple of registered outcome strings.
        """
        return tuple(self._outcomes)

    ### Userdata API
    def register_io_keys(self, keys):
        """Add keys to the set of keys from which this state may read and write.
        @type keys: list of str
        @param keys: List of keys which may be read from and written to when this
        state is active.
        """
        self._input_keys = self._input_keys.union(keys)
        self._output_keys = self._output_keys.union(keys)

    def register_input_keys(self, keys):
        """Add keys to the set of keys from which this state may read.
        @type keys: list of str
        @param keys: List of keys which may be read from when this state is
        active.
        """
        self._input_keys = self._input_keys.union(keys)

    def get_registered_input_keys(self):
        """Get a tuple of registered input keys."""
        return tuple(self._input_keys)

    def register_output_keys(self, keys):
        """Add keys to the set of keys to which this state may write.
        @type keys: list of str
        @param keys: List of keys which may be written to when this state is
        active.
        """
        self._output_keys = self._output_keys.union(keys)

    def get_registered_output_keys(self):
        """Get a tuple of registered output keys."""
        return tuple(self._output_keys)

    ### Preemption interface
    def request_preempt(self):
        """Sets preempt_requested to True"""
        self._preempt_requested = True

    def service_preempt(self):
        """Sets preempt_requested to False"""
        self._preempt_requested = False

    def recall_preempt(self):
        """Sets preempt_requested to False"""
        self._preempt_requested = False

    def preempt_requested(self):
        """True if a preempt has been requested."""
        return self._preempt_requested

class CBState(State):
    def __init__(self, cb, cb_args=[], cb_kwargs={}, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        """Create s state from a single function.

        @type outcomes: list of str
        @param outcomes: Custom outcomes for this state.

        @type input_keys: list of str
        @param input_keys: The userdata keys from which this state might read
        at runtime. 

        @type output_keys: list of str
        @param output_keys: The userdata keys to which this state might write
        at runtime.

        @type io_keys: list of str
        @param io_keys: The userdata keys to which this state might write or
        from which it might read at runtime.
        """
        State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self._cb = cb
        self._cb_args = cb_args
        self._cb_kwargs = cb_kwargs

        if smach.util.has_smach_interface(cb):
            self._cb_input_keys = cb.get_registered_input_keys()
            self._cb_output_keys = cb.get_registered_output_keys()
            self._cb_outcomes = cb.get_registered_outcomes()

            self.register_input_keys(self._cb_input_keys)
            self.register_output_keys(self._cb_output_keys)
            self.register_outcomes(self._cb_outcomes)

    def execute(self, ud):
        return self._cb(ud, *self._cb_args, **self._cb_kwargs)

