#!/usr/bin/env python
# http://docs.ros.org/jade/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cluster_analysis', 'calibration'],
    package_dir={'': 'python'}
)

setup(**setup_args)
