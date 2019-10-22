#!/usr/bin/env python
# http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['scripts/dummy_gripper_action_server.py', 'scripts/HandTcpInterface.py',
             'scripts/RobotiqJointStatePublisher.py'],
    packages=['tbf_gripper_hand'],
    package_dir={'': 'python'}
)

setup(**d)
