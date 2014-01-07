
import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback

import smach
from smach.state import *

__all__ = ['ServiceState']

class ServiceState(smach.State):
    """State for calling a service."""
    def __init__(self,
            # Service info
            service_name,
            service_spec,
            # Request Policy
            request = None,
            request_cb = None,
            request_cb_args = [],
            request_cb_kwargs = {},
            request_key = None,
            request_slots = [],
            # Response Policy
            response_cb = None,
            response_cb_args = [],
            response_cb_kwargs = {},
            response_key = None,
            response_slots = [],
            # Keys
            input_keys = [],
            output_keys = [],
            outcomes = [],
            ):

        smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'])

        # Store Service info
        self._service_name = service_name
        self._service_spec = service_spec

        self._proxy = None

        # Store request policy
        if request is None:
            self._request = service_spec._request_class()
        else:
            self._request = request

        
        if request_cb is not None and not hasattr(request_cb, '__call__'):
            raise smach.InvalidStateError("Request callback object given to ServiceState that IS NOT a function object")

        self._request_cb = request_cb
        self._request_cb_args = request_cb_args
        self._request_cb_kwargs = request_cb_kwargs
        if smach.has_smach_interface(request_cb):
            self._request_cb_input_keys = request_cb.get_registered_input_keys()
            self._request_cb_output_keys = request_cb.get_registered_output_keys()

            self.register_input_keys(self._request_cb_input_keys)
            self.register_output_keys(self._request_cb_output_keys)
        else:
            self._request_cb_input_keys = input_keys
            self._request_cb_output_keys = output_keys

        self._request_key = request_key
        if request_key is not None:
            self.register_input_keys([request_key])

        self._request_slots = request_slots
        self.register_input_keys(request_slots)

        # Store response policy
        if response_cb is not None and not hasattr(response_cb, '__call__'):
            raise smach.InvalidStateError("Response callback object given to ServiceState that IS NOT a function object")

        self._response_cb = response_cb
        self._response_cb_args = response_cb_args
        self._response_cb_kwargs = response_cb_kwargs
        if smach.has_smach_interface(response_cb):
            self._response_cb_input_keys = response_cb.get_registered_input_keys()
            self._response_cb_output_keys = response_cb.get_registered_output_keys()
            self._response_cb_outcomes = response_cb.get_registered_outcomes()

            self.register_input_keys(self._response_cb_input_keys)
            self.register_output_keys(self._response_cb_output_keys)
            self.register_outcomes(self._response_cb_outcomes)
        else:
            self._response_cb_input_keys = input_keys
            self._response_cb_output_keys = output_keys
            self._response_cb_outcomes = outcomes

        # Register additional input and output keys
        self.register_input_keys(input_keys)
        self.register_output_keys(output_keys)
        self.register_outcomes(outcomes)

        self._response_key = response_key
        if response_key is not None:
            self.register_output_keys([response_key])

        self._response_slots = response_slots
        self.register_output_keys(response_slots)

    def execute(self, ud):
        """Execute service"""
        # Check for preemption before executing
        if self.preempt_requested():
            rospy.loginfo("Preempting %s before sending request." % self._service_name)
            self.service_preempt()
            return 'preempted'

        # Make sure we're connected to the service
        try:
            while not rospy.is_shutdown() and self._proxy is None:
                if self.preempt_requested():
                    rospy.loginfo("Preempting while waiting for service '%s'." % self._service_name)
                    self.service_preempt()
                    return 'preempted'
                try:
                    rospy.wait_for_service(self._service_name,1.0)
                    self._proxy = rospy.ServiceProxy(self._service_name, self._service_spec)
                    rospy.logdebug("Connected to service '%s'" % self._service_name)
                except rospy.ROSException as ex:
                    rospy.logwarn("Still waiting for service '%s'..." % self._service_name)
        except:
            rospy.logwarn("Terminated while waiting for service '%s'." % self._service_name)
            return 'aborted'

        # Grab request key if set
        if self._request_key is not None:
            if self._request_key in ud:
                self._request = ud[self._request_key]
            else:
                rospy.logerr("Requested request key '%s' not in userdata struture. Available keys are: %s" % (self._request_key, str(list(ud.keys()))))
                return 'aborted'

        # Write request fields from userdata if set
        for key in self._request_slots:
            if key in ud:
                setattr(self._request,key,ud[key])
            else:
                rospy.logerr("Requested request slot key '%s' is not in userdata strcture. Available keys are: %s" % (key, str(list(ud.keys()))))
                return 'aborted'

        # Call user-supplied callback, if set, to get a request
        if self._request_cb is not None:
            try:
                request_update = self._request_cb(
                        smach.Remapper(
                                ud,
                                self._request_cb_input_keys,
                                self._request_cb_output_keys,
                                []),
                        self._request,
                        *self._request_cb_args,
                        **self._request_cb_kwargs)
                if request_update is not None:
                    self._request = request_update
            except:
                rospy.logerr("Could not execute request callback: "+traceback.format_exc())
                return 'aborted'

        if self._request is None:
            rospy.logerr("Attempting to call service "+self._service_name+" with no request")
            return 'aborted'

        # Call service
        # Abandon hope, all ye who enter here
        try:
            rospy.logdebug("Calling service %s with request:\n%s" % (self._service_name, str(self._request)))
            self._response = self._proxy(self._request)
        except rospy.ServiceException as ex:
            rospy.logerr("Exception when calling service '%s': %s" % (self._service_name, str(ex)))
            return 'aborted'

        # Call response callback if it's set
        response_cb_outcome = None
        if self._response_cb is not None:
            try:
                response_cb_outcome = self._response_cb(
                        smach.Remapper(
                                ud,
                                self._response_cb_input_keys,
                                self._response_cb_output_keys,
                                []),
                        self._response,
                        *self._response_cb_args,
                        **self._response_cb_kwargs)
                if response_cb_outcome is not None and response_cb_outcome not in self.get_registered_outcomes():
                    rospy.logerr("Result callback for service "+self._service_name+", "+str(self._response_cb)+" was not registered with the response_cb_outcomes argument. The response callback returned '"+str(response_cb_outcome)+"' but the only registered outcomes are: "+str(self.get_registered_outcomes()))
                    return 'aborted'
            except:
                rospy.logerr("Could not execute response callback: "+traceback.format_exc())
                return 'aborted'

        if self._response_key is not None:
            ud[self._response_key] = self._response

        for key in self._response_slots:
            ud[key] = getattr(self._response,key)

        if response_cb_outcome is not None:
            return response_cb_outcome

        return 'succeeded'
