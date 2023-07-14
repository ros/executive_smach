#!/usr/bin/env python3
import os
from glob import glob
from setuptools import setup

package_name = 'smach'

setup(
    name=package_name,
    version='3.0.3',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac I. Y. Saito',
    maintainer_email='gm130s@gmail.com',
    description='''
        SMACH is a task-level architecture for rapidly creating complex robot
        behavior. At its core, SMACH is a ROS-independent Python library to build
        hierarchical state machines. SMACH is a new library that takes advantage of
        very old concepts in order to quickly create robust robot behavior with
        maintainable and modular code.''',
    license='BSD',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)
