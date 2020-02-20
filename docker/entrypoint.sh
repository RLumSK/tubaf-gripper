#!/bin/bash

# setup ros environment
source "/opt/ros/kinetic/setup.bash"

# exec /pkg/scripts/evaluate_bag.py -p /in/bag/buero.bag
# usage: evaluate_bag.py [-h] [-r] [-p PATH] [-pt PUB_TOPIC]
#                        [-ot OBSTACLE_TOPIC] [-ft FLOOR_TOPIC] [-ss SUB_SAMPLE]
#                        [-nb N_BINS] [-mcr MC_RASTER]

# Analyse a BAG-file and plot results

# optional arguments:
#   -h, --help            show this help message and exit
#   -r, --use_ros         use parameters stored on the ROS Paramter server, if
#                         false commandline arguments could be used instead
#   -p PATH, --path PATH  Path of the BAG-file
#   -pt PUB_TOPIC, --pub_topic PUB_TOPIC
#                         Publisher topic
#   -ot OBSTACLE_TOPIC, --obstacle_topic OBSTACLE_TOPIC
#                         Topic name of the obstacles
#   -ft FLOOR_TOPIC, --floor_topic FLOOR_TOPIC
#                         Topic name of the floor
#   -ss SUB_SAMPLE, --sub_sample SUB_SAMPLE
#                         Subsample rate
#   -nb N_BINS, --n_bins N_BINS
#                         [KDE] Number of Bins used
#   -mcr MC_RASTER, --mc_raster MC_RASTER
#                         [MC] Number of x and y line
exec "$@"