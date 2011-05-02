
import smach
import threading


__all__ = ['is_shutdown','set_shutdown_cb',\
        'cb_interface','has_smach_interface','CBInterface']

def is_shutdown():
    return False

def set_shutdown_check(cb):
    smach.is_shutdown = cb

def has_smach_interface(obj):
    """Returns True if the object has SMACH interface accessors."""
    return hasattr(obj,'get_registered_input_keys')\
            and hasattr(obj,'get_registered_output_keys')\
            and hasattr(obj,'get_registered_outcomes')

# Callback decorator for describing userdata
class cb_interface(object):
    def __init__(self, outcomes=[], input_keys=[], output_keys=[]):
        self._outcomes = outcomes
        self._input_keys = input_keys
        self._output_keys = output_keys
    
    def __call__(self, cb):
        return CBInterface(cb, self._outcomes, self._input_keys, self._output_keys)
class CBInterface(object):
    """Decorator to describe the extension of a state's SMACH userdata and outcome interface.
    
    Some SMACH states can be extended with the use of user callbacks. Since
    the SMACH interface and SMACH userdata are strictly controlled, the ways in
    which these callbacks interact with SMACH must be delcared. This decorator
    allows this information to be attached to a given callback function.

    If a callback adds a potential outcome to a state, suppose 'critical_failure',
    then one could write this when defining the callback:

    >>> import smach
    >>> @smach.cb_interface(outcomes=['critical_failure'])
    >>> def my_cb(x,y,z):
    >>>     # User code
    >>>     return 'critical_failure'

    Suppose a state retrieves data that it passes into a callback. If the user
    wants to take that data and put some of all of it into userdata, this
    interface must be declared. In this case, the user could write:

    >>> import smach
    >>> @smach.cb_interface(output_keys=['processed_res'])
    >>> def my_cb(ud, data):
    >>>     ud.processed_res = data

    """
    def __init__(self, cb, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        """Describe callback SMACH interface.

        @type outcomes: array of strings
        @param outcomes: Custom outcomes for this state.

        @type input_keys: array of strings
        @param input_keys: The userdata keys from which this state might read
        at runtime. 

        @type output_keys: array of strings
        @param output_keys: The userdata keys to which this state might write
        at runtime.

        @type io_keys: array of strings
        @param io_keys: The userdata keys to which this state might write or
        from which it might read at runtime.
        """

        self._input_keys = set(input_keys)
        self._input_keys.union(io_keys)

        self._output_keys = set(output_keys)
        self._output_keys.union(io_keys)

        self._outcomes = outcomes

        self._cb = cb

    def __call__(self, *args, **kwargs):
        return self._cb(*args, **kwargs)

    ### SMACH Interface API
    def get_registered_input_keys(self):
        """Get a tuple of registered input keys."""
        return tuple(self._input_keys)
    def get_registered_output_keys(self):
        """Get a tuple of registered output keys."""
        return tuple(self._output_keys)
    def get_registered_outcomes(self):
        """Get a list of registered outcomes.
        @rtype: tuple of string
        @return: Tuple of registered outcome strings.
        """
        return tuple(self._outcomes)

