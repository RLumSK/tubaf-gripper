#!/usr/bin/env bash

# setup ros environment
source "/opt/ros/kinetic/setup.bash"

exec /pkg/scripts/evaluate_bag.py --help

#exec "$@"