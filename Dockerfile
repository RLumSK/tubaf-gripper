# syntax=docker/dockerfile:experimental
# FROM julius as intermediate
# # Build: DOCKER_BUILDKIT=1 docker build --ssh default -t set_pose_evaluation .

# ARG WS=/julius_ws/src
# ARG GIT_REPO=git.informatik.tu-freiberg.de

# # RUN rm -rf $WS/tubaf_gripper
# # RUN --mount=type=ssh git clone --recurse-submodules --branch=pose_generator --depth=1 git@$GIT_REPO:ros/tubaf_gripper.git $WS/tubaf_gripper
# WORKDIR $WS/tubaf_gripper
# RUN git checkout pose_generator


# ########################################
FROM ros:kinetic-robot
LABEL maintainer="grehl" mail="Steve.Grehl@informatik.tu-freiberg.de" company="TU Bergakademie Freiberg"

## Install python modules
WORKDIR /tmp
RUN rm -rf /var/lib/apt/lists/* \
 && apt-get update \
 && apt-get install -y \
 	python3-pip \
 	ros-kinetic-object-recognition-msgs \
 	ros-kinetic-visualization-msgs \
 	ros-kinetic-geometry-msgs \
 	ros-kinetic-message-filters \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --upgrade pip
RUN pip3 install scipy sklearn pyyaml rospkg matplotlib pandas
RUN pip3 install matplotlib2tikz tikzplotlib

COPY ./docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

# create filestructure
VOLUME /out/plots
VOLUME /in/bag

RUN mkdir /pkg
COPY ./tbf_gripper_autonomy /pkg
ENV PATH=${PATH}:/pkg/scripts
ENV PYTHONPATH=${PYTHONPATH}:/pkg/python/

# CMD ["bash"]
WORKDIR /pkg
ENTRYPOINT ["/entrypoint.sh"]
#					   FROM : TO									MATCH USER
# docker run --volume $(pwd)/:/in/bag --volume $(pwd)/plots:/out/plots -u $(id -u):$(id -g) set_pose_evaluation buero.bag
# docker run --volume $(pwd)/docker:/in/bag --volume $(pwd)/plots:/out/plots -u $(id -u):$(id -g) set_pose_evaluation evaluate_bag.py -p /in/bag/buero.bag