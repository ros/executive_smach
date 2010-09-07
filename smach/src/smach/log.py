
import smach

__all__ = ['set_loggers','loginfo','logwarn','logerr','logdebug']

def loginfo(msg):
    print("[  INFO ] : "+str(msg))

def logwarn(msg):
    print("[  WARN ] : "+str(msg))

def logdebug(msg):
    print("[ DEBUG ] : "+str(msg))

def logerr(msg):
    print("[ ERROR ] : "+str(msg))

def set_loggers(info,warn,debug,error):
    """Override the SMACH logging functions."""
    smach.loginfo = info
    smach.logwarn = warn
    smach.logdebug = debug
    smach.logerr = error

