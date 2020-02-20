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
FROM julius
LABEL maintainer="grehl" mail="Steve.Grehl@informatik.tu-freiberg.de" company="TU Bergakademie Freiberg"

ARG WS=/julius_ws/src

# create filestructure
VOLUME /out/plots
VOLUME /in/bag

# Get correct branch@tubaf_gripper
WORKDIR $WS/tubaf_gripper
RUN git pull && git checkout pose_generator

# build ros package source
WORKDIR $WS/..
RUN catkin config \
      --extend /opt/ros/kinetic && \
    catkin build tubaf_gripper

## Install python modules
WORKDIR /tmp
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install scipy sklearn

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
#					   FROM : TO									MATCH USER
# docker run --volume $(pwd)/:/in/bag --volume $(pwd):/out/plots -u $(id -u):$(id -g) set_pose_evaluation buero.bag
# roscore &; rosrun tbf_gripper_autonomy evaluate_bag.py _plot_dir:="/plots" _bag:="/bag/buero.bag"