# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE

# ROS imports
import roslib; roslib.load_manifest('smach_ros')
import rospy

# SMACH
import smach

__all__ = ['set_preempt_handler',
        'start',
        'ActionServerWrapper',
        'IntrospectionClient','IntrospectionServer',
        'SimpleActionState',
        'ServiceState',
        'MonitorState',
        'ConditionState']

# Setup smach-ros interface
smach.set_loggers(
        rospy.loginfo,
        rospy.logwarn,
        rospy.logdebug,
        rospy.logerr)

smach.set_shutdown_check(rospy.is_shutdown)
smach.set_shutdown_handler(rospy.on_shutdown)

### Core classes
from util import set_preempt_handler, start

### Top-level Containers / Wrappers
from action_server_wrapper import ActionServerWrapper
from introspection import IntrospectionClient, IntrospectionServer

### State Classes
from simple_action_state import SimpleActionState
from service_state import ServiceState
from monitor_state import MonitorState
from condition_state import ConditionState
