#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading

__all__ = ['SmachNode']

class SmachNode(Node):
    """
    A ROS2 Node executing a SMACH StateMachine
    """
    def __init__(self, node_name, **kwargs):
        Node.__init__(self, node_name, **kwargs)
        self.__executor = SingleThreadedExecutor()

    def start(self):
        def spin():
            rclpy.spin(self, executor=self.__executor)
        self.__spinner = threading.Thread(target=spin)
        self.__spinner.start()

    def join(self):
        self.__spinner.join()
