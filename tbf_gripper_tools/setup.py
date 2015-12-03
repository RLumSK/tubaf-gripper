# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tbf_gripper_tools'],
    package_dir={'': 'src'},
)

setup(**d)