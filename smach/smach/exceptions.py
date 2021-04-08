
import smach

__all__ = ['SmachError',
           'InvalidTransitionError',
           'InvalidStateError',
           'InvalidConstructionError',
           'InvalidUserCodeError']


class SmachError(Exception):
    """Exception printing to console on instantiation"""
    def __init__(self, message):
        smach.logerr(self.__class__.__name__ + ": " + message)
        Exception.__init__(self, message)


class InvalidTransitionError(SmachError):
    def __init__(self, message):
        SmachError.__init__(self, message)

class InvalidStateError(SmachError):
    def __init__(self, message):
        SmachError.__init__(self, message)

class InvalidUserCodeError(SmachError):
    def __init__(self, message):
        SmachError.__init__(self, message)

class InvalidConstructionError(SmachError):
    def __init__(self, message):
        SmachError.__init__(self, message)
