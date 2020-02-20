#!/usr/bin/env bash

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/julius_ws/devel/setup.bash"

roscore &
rosrun tbf_gripper_autonomy evaluate_bag.py _plot_dir:="/out/plots" _bag:="/in/bag/$1"

#exec "$@"