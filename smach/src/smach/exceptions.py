
import smach

__all__ = ['InvalidTransitionError',
        'InvalidStateError',
        'InvalidConstructionError',
        'InvalidUserCodeError']

class InvalidTransitionError():
    def __init__(self,message):
        smach.logerr("InvalidTransitionError: "+message)
        self.message = message

class InvalidStateError():
    def __init__(self,message):
        smach.logerr("InvalidStateError: "+message)
        self.message = message

class InvalidUserCodeError():
    def __init__(self,message):
        smach.logerr("InvalidUserCodeError: "+message)
        self.message = message

class InvalidConstructionError():
    def __init__(self,message):
        smach.logerr("InvalidConstructionError: "+message)
        self.message = message
