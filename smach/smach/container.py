
import traceback
import threading
from contextlib import contextmanager

import smach

__all__ = ['Container']

class Container(smach.state.State):
    """Smach container interface.

    This provides an interface for hooking into smach containers. This includes
    methods to get and set state, as well as provide start / transition /
    termination callback storage and registration utilities.

    Note that it is up to the implementation of the containers both when the 
    callbacks are called as well as what arguments are given to them.

    Callback semantics:
     - Start: Called when a container is entered
     - Transition: Called when a container's state changes
     - Termination: Called when a container is left
    """

    ### Class members
    _construction_stack = []
    _construction_lock = threading.RLock()
    _context_kwargs = []

    def __init__(self,
            outcomes=[],
            input_keys=[],
            output_keys=[]):
        """Initializes callback lists as empty lists."""
        smach.state.State.__init__(self, outcomes, input_keys, output_keys)

        self.userdata = smach.UserData()
        """Userdata to be passed to child states."""

        # Callback lists
        self._start_cbs = []
        self._transition_cbs = []
        self._termination_cbs = []

    def __getitem__(self, key):
        """Access child state by key.
        @rtype: L{smach.State}
        @returns: Child state with label equal to key
        """
        raise NotImplementedError()

    def get_children(self):
        """Get the children of this container.
        This is empty for leaf states.

        @rtype: dict of string: State
        @return: The sub-states of this container.
        """
        raise NotImplementedError()
    
    def set_initial_state(self, initial_states, userdata):
        """Set initial active states of a container.
        
        @type initial_states: list of string
        @param initial_states: A description of the initial active state of this
        container.
        
        @type userdata: L{UserData}
        @param userdata: Initial userdata for this container.
        """
        raise NotImplementedError()

    def get_initial_states(self):
        """Get the initial states description.
        
        @rtype: list of string
        """
        raise NotImplementedError()

    def get_active_states(self):
        """Get a description of the current states.
        Note that this is specific to container implementation.

        @rtype: list of string
        """
        raise NotImplementedError()

    def get_internal_edges(self):
        """Get the internal outcome edges of this container.
        Get a list of 3-tuples (OUTCOME, LABEL_FROM, LABEL_TO) which correspond
        to transitions inside this container.

        @rtype: list of 3-tuple
        """
        raise NotImplementedError()

    def check_consistency(self):
        """Check consistency of this container."""
        raise NotImplementedError()

    ### Automatic Data passing
    def _copy_input_keys(self, parent_ud, ud):
        if parent_ud is not None:
            input_keys = self.get_registered_input_keys()
            for ik in input_keys:
                try:
                    ud[ik] = parent_ud[ik]
                except KeyError:
                    smach.logwarn("Attempting to copy input key '%s', but this key does not exist." % ik)

    def _copy_output_keys(self, ud, parent_ud):
        if parent_ud is not None:
            output_keys = self.get_registered_output_keys()
            for ok in output_keys:
                try:
                    parent_ud[ok] = ud[ok]
                except KeyError:
                    smach.logwarn("Attempting to copy output key '%s', but this key does not exist." % ok)

    ### Callback registreation methods
    def register_start_cb(self, start_cb, cb_args=[]):
        """Adds a start callback to this container.
        Start callbacks receive arguments:
         - userdata 
         - local_userdata
         - initial_states
         - *cb_args
        """
        self._start_cbs.append((start_cb,cb_args))

    def register_transition_cb(self, transition_cb, cb_args=[]):
        """Adds a transition callback to this container.
        Transition callbacks receive arguments:
         - userdata 
         - local_userdata
         - active_states
         - *cb_args
        """
        self._transition_cbs.append((transition_cb,cb_args))

    def register_termination_cb(self, termination_cb, cb_args=[]):
        """Adds a termination callback to this state machine.
        Termination callbacks receive arguments:
         - userdata 
         - local_userdata
         - terminal_states
         - container_outcome
         - *cb_args
        """
        self._termination_cbs.append((termination_cb, cb_args)) 

    def call_start_cbs(self):
        """Calls the registered start callbacks.
        Callback functions are called with two arguments in addition to any
        user-supplied arguments:
         - userdata
         - a list of initial states
         """
        try:
            for (cb,args) in self._start_cbs:
                cb(self.userdata, self.get_initial_states(), *args)
        except:
            smach.logerr("Could not execute start callback: "+traceback.format_exc())

    def call_transition_cbs(self):
        """Calls the registered transition callbacks.
        Callback functions are called with two arguments in addition to any
        user-supplied arguments:
         - userdata
         - a list of active states
         """
        try:
            for (cb,args) in self._transition_cbs:
                cb(self.userdata, self.get_active_states(), *args)
        except:
            smach.logerr("Could not execute transition callback: "+traceback.format_exc())

    def call_termination_cbs(self, terminal_states, outcome):
        """Calls the registered termination callbacks.
        Callback functions are called with three arguments in addition to any
        user-supplied arguments:
         - userdata
         - a list of terminal states
         - the outcome of this container
        """
        try:
            for (cb,args) in self._termination_cbs:
                cb(self.userdata, terminal_states, outcome, *args)
        except:
            smach.logerr("Could not execute termination callback: "+traceback.format_exc())


    # Context manager methods
    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            return self.close()
        else:
            if exc_type != smach.InvalidStateError and exc_type != smach.InvalidTransitionError:
                smach.logerr("Error raised during SMACH container construction: \n" + "\n".join(traceback.format_exception(exc_type, exc_val, exc_tb)))

    @contextmanager
    def opened(self, **kwargs):
        """Context manager method for opening a smach container."""
        self.open()
        prev_kwargs = Container._context_kwargs
        Container._context_kwargs = kwargs
        try:
            yield self
        finally:
            Container._context_kwargs = prev_kwargs 
            self.close()

    def open(self):
        """Opens this container for modification.

        This appends the container to the construction stack and locks the
        reentrant lock if it is a valid container to open."""

        # Push this container onto the construction stack
        Container._construction_stack.append(self)
        Container._construction_lock.acquire()

    def close(self):
        """Close the container."""
        # Make sure this container is the currently open container
        if len(Container._construction_stack) > 0:
            if self != Container._construction_stack[-1]:
                raise smach.InvalidStateError('Attempting to close a container that is not currently open.')

        # Pop this container off the construction stack
        Container._construction_stack.pop()
        Container._construction_lock.release()

        # Check consistency of container, post-construction
        try:
            self.check_consistency()
        except (smach.InvalidStateError, smach.InvalidTransitionError):
            smach.logerr("Container consistency check failed.")

    def is_opened(self):
        """Returns True if this container is currently opened for construction.
        @rtype: bool
        """
        return len(Container._construction_stack) > 0 and self == Container._construction_stack[-1]

    def assert_opened(self,msg=''):
        if not self.is_opened():
            raise smach.InvalidConstructionError(msg)

    @staticmethod
    def _any_containers_opened():
        """Returns True if any containers are opened."""
        if len(Container._construction_stack) > 0:
            return True
        return False

    @classmethod
    def _currently_opened_container(cls):
        """Get the currently opened container.
        
        This also asserts that the open container is of type cls.
        """
        if Container._any_containers_opened():
            opened_container = Container._construction_stack[-1]
            if not isinstance(opened_container, cls):
                raise smach.InvalidStateError('Attempting to call a %s construction method without having opened a %s.' % (cls, cls))
            return opened_container
        else:
            raise smach.InvalidStateError('Attempting to access the currently opened container, but no container is opened.')
