#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock
from std_msgs.msg import Header

import base64
import time
import pickle
import threading
from functools import partial

import smach

from smach_msgs.msg import SmachContainerStatus,SmachContainerInitialStatusCmd,SmachContainerStructure

__all__ = ['IntrospectionClient','IntrospectionServer']

# Topic names
STATUS_TOPIC = '/smach/container_status'
INIT_TOPIC = '/smach/container_init'
STRUCTURE_TOPIC = '/smach/container_structure'

class IntrospectionClient(Node):
    def __init__(self, node_name='introspection_client', **kwargs):
        Node.__init__(self, node_name, **kwargs)

    def get_servers(self):
        """Get the base names that are broadcasting smach states."""

        # Get the currently broadcasted smach introspection topics
        topics = [top for top, typ in self.get_topic_names_and_types() if 'smach_msgs/msg/SmachContainerStatus' in typ]

        return [t[:t.rfind(STATUS_TOPIC)] for t in topics]

    def set_initial_state(self,
            server,
            path,
            initial_states,
            initial_userdata = smach.UserData(),
            timeout = None):
        """Set the initial state of a smach server.

        @type server: string
        @param server: The name of the introspection server to which this client
        should connect.

        @type path: string
        @param path: The path to the target container in the state machine.

        @type initial_states: list of string
        @param inital_state: The state the target container should take when it
        starts. This is as list of at least one state label.

        @type initial_userdata: UserData
        @param initial_userdata: The userdata to inject into the target container.

        @type timeout: rclpy.time.Duration
        @param timeout: Timeout for this call. If this is set to None, it will not
        block, and the initial state may not be set before the target state machine
        goes active.
        """

        # Construct initial state command
        initial_status_msg = SmachContainerInitialStatusCmd(
                path = path,
                initial_states = initial_states,
                local_data = base64.b64encode(pickle.dumps(initial_userdata._data, 2)))

        # A status message to receive confirmation that the state was set properly
        msg_response = SmachContainerStatus()

        # Define a local callback to just stuff a local message
        def local_cb(msg_response, msg):
            self.get_logger().debug("Received status response: "+str(msg))
            msg_response.path = msg.path
            msg_response.initial_states = msg.initial_states
            msg_response.local_data = msg.local_data

        # Create a subscriber to verify the request went through
        state_sub = self.create_subscription(
                SmachContainerStatus,
                server+STATUS_TOPIC,
                partial(local_cb, msg_response),
                1)

        # Create a publisher to send the command
        self.get_logger().debug("Sending initial state command: "+str(initial_status_msg.path)+" on topic '"+server+INIT_TOPIC+"'")
        init_pub = self.create_publisher(
                SmachContainerInitialStatusCmd,
                server+INIT_TOPIC,
                1)
        init_pub.publish(initial_status_msg)

        start_time = self.get_clock().now()

        # Block until we get a new state back
        if timeout is not None:
            while self.get_clock().now() - start_time < timeout:
                # Send the initial state command
                init_pub.publish(initial_status_msg)

                # Filter messages that are from other containers
                if msg_response.path == path:
                    # Check if the heartbeat came back to match
                    state_match = all([s in msg_response.initial_states for s in initial_states])
                    local_data = smach.UserData()
                    local_data._data = pickle.loads(base64.b64decode(msg_response.local_data))
                    ud_match = all([\
                            (key in local_data and local_data._data[key] == initial_userdata._data[key])\
                            for key in initial_userdata._data])

                    self.get_logger().debug("STATE MATCH: "+str(state_match)+", UD_MATCH: "+str(ud_match))

                    if state_match and ud_match:
                        return True
                time.sleep(0.25)
            return False

class ContainerProxy():
    """Smach Container Introspection proxy.

    This class is used as a container for introspection and debugging.
    """
    def __init__(self, server, server_name, container, path, update_rate=2.0):
        """Constructor for tree-wide data structure.
        """
        self._path = path
        self._container = container
        self._status_pub_lock = threading.Lock()
        self._server_node = server

        # Advertise init service
        self._init_cmd = self._server_node.create_subscription(
                SmachContainerInitialStatusCmd,
                server_name + INIT_TOPIC,
                self._init_cmd_cb,
                1)

        # Advertise structure publisher
        self._structure_pub = self._server_node.create_publisher(
                SmachContainerStructure,
                server_name + STRUCTURE_TOPIC,
                1)

        # Advertise status publisher
        self._status_pub = self._server_node.create_publisher(
                SmachContainerStatus,
                server_name + STATUS_TOPIC,
                1)

        # Set transition callback
        container.register_transition_cb(self._transition_cb)

        # Create thread to constantly publish
        self._status_pub_timer = self._server_node.create_timer(update_rate, self._status_pub_loop)
        self._structure_pub_timer = self._server_node.create_timer(update_rate, self._structure_pub_loop)

        self._keep_running = False

    def start(self):
        self._keep_running = True

    def stop(self):
        self._keep_running = False

    def _status_pub_loop(self):
        """Publish the status heartbeats."""
        self._publish_status('HEARTBEAT')

    def _structure_pub_loop(self):
        """Publish the structure heartbeats."""
        self._publish_structure('HEARTBEAT')

    def _publish_structure(self, info_str=''):
        path = self._path
        children = list(self._container.get_children().keys())

        internal_outcomes = []
        outcomes_from = []
        outcomes_to = []
        for (outcome, from_label, to_label) in self._container.get_internal_edges():
            internal_outcomes.append(str(outcome))
            outcomes_from.append(str(from_label))
            outcomes_to.append(str(to_label))
        container_outcomes = self._container.get_registered_outcomes()

        # Construct structure message
        structure_msg = SmachContainerStructure(
                header=Header(stamp = ROSClock().now().to_msg()),
                path=path,
                children=children,
                internal_outcomes=internal_outcomes,
                outcomes_from=outcomes_from,
                outcomes_to=outcomes_to,
                container_outcomes=container_outcomes)
        try:
            self._structure_pub.publish(structure_msg)
        except:
            if rclpy.ok():
                self._server_node.get_logger().error("Publishing SMACH introspection structure message failed.")

    def _publish_status(self, info_str=''):
        """Publish current state of this container."""
        # Construct messages
        with self._status_pub_lock:
            path = self._path

            # Transform userdata to dictionary for pickling
            keys = list(self._container.userdata.keys())
            data = {key: self._container.userdata[key] for key in keys}
            state_msg = SmachContainerStatus(
                    header=Header(stamp = ROSClock().now().to_msg()),
                    path=path,
                    initial_states=self._container.get_initial_states(),
                    active_states=self._container.get_active_states(),
                    local_data=base64.b64encode(pickle.dumps(data, 2)),
                    info=info_str)
            # Publish message
            self._status_pub.publish(state_msg)

    ### Transition reporting
    def _transition_cb(self, *args, **kwargs):
        """Transition callback, passed to all internal nodes in the tree.
        This callback locks an internal mutex, preventing any hooked transitions
        from occurring while we're walking the tree.
        """
        info_str = (str(args) + ', ' + str(kwargs))
        self._server_node.get_logger().debug("Transitioning: "+info_str)
        self._publish_status(info_str)

    def _init_cmd_cb(self, msg):
        """Initialize a tree's state and userdata."""
        initial_states = msg.initial_states
        local_data = msg.local_data

        # Check if this init message is directed at this path
        self._server_node.get_logger().debug('Received init message for path: '+msg.path+' to '+str(initial_states))
        if msg.path == self._path:
            if all(s in self._container.get_children() for s in initial_states):
                ud = smach.UserData()
                ud._data = pickle.loads(base64.b64decode(msg.local_data))
                self._server_node.get_logger().debug("Setting initial state in smach path: '"+self._path+"' to '"+str(initial_states)+"' with userdata: "+str(ud._data))

                # Set the initial state
                self._container.set_initial_state(
                        initial_states,
                        ud)
                # Publish initial state
                self._publish_status("REMOTE_INIT")
            else:
                self._server_node.get_logger().error("Attempting to set initial state in container '"+self._path+"' to '"+str(initial_states)+"', but this container only has states: "+str(self._container.get_children()))


class IntrospectionServer(Node):
    """Server for providing introspection and control for smach."""
    def __init__(self, server_name, state, path):
        """Traverse the smach tree starting at root, and construct introspection
        proxies for getting and setting debug state."""
        Node.__init__(self, server_name)

        # A list of introspection proxies
        self._proxies = []

        # Store args
        self._server_name = server_name
        self._state = state
        self._path = path

    def start(self):
        # Construct proxies
        self.construct(self._server_name, self._state, self._path)

    def stop(self):
        for proxy in self._proxies:
            proxy.stop()

    def construct(self, server_name, state, path):
        """Recursively construct proxies to containers."""
        # Construct a new proxy
        proxy = ContainerProxy(self, server_name, state, path)

        if path == '/':
            path = ''

        # Get a list of children that are also containers
        for (label, child) in state.get_children().items():
            # If this is also a container, recurse into it
            if isinstance(child, smach.container.Container):
                self.construct(server_name, child, path+'/'+label)

        # Publish initial state
        proxy._publish_status("Initial state")

        # Start publisher threads
        proxy.start()

        # Store the proxy
        self._proxies.append(proxy)

    def clear(self):
        """Clear all proxies in this server."""
        self._proxies = []
