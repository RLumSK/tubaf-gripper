#!/usr/bin/env bash

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/julius_ws/devel/setup.bash"

exec "$@"