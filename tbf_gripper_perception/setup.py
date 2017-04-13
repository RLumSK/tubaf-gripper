#!/usr/bin/env python
# http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
#    scripts=['scripts/marker_reg.py', 'scripts/model_tracker.py', 'scripts/rot_test.py', 'scripts/pl_interface.py'],
    packages=['cluster_analysis'],
    package_dir={'': 'python'}
)

setup(**d)
