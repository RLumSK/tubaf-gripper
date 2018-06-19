# http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tbf_gripper_rqt', 'tbf_gripper_rviz'],
    package_dir={'': 'python'},
)

setup(**d)