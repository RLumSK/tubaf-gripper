# syntax=docker/dockerfile:experimental
FROM julius as intermediate
# Build: DOCKER_BUILDKIT=1 docker build --ssh default -t set_pose_evaluation .  

ARG WS=/julius_ws/src
ARG GIT_REPO=git.informatik.tu-freiberg.de

RUN ssh-keygen -f "/root/.ssh/known_hosts" -R 139.20.16.39
RUN rm -rf $WS/tubaf_gripper
RUN --mount=type=ssh git clone --recurse-submodules --branch=pose_generator --depth=1 git@$GIT_REPO:ros/tubaf_gripper.git $WS/tubaf_gripper

########################################
FROM julius
LABEL maintainer="grehl" mail="Steve.Grehl@informatik.tu-freiberg.de" company="TU Bergakademie Freiberg"

ARG WS=/julius_ws/src
COPY --from=intermediate $WS/tubaf_gripper $WS/tubaf_gripper

WORKDIR $WS/..

RUN mkdir /plots
RUN mkdir /bag
COPY buero.bag /bag/

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
# roscore | rosrun tbf_gripper_autonomy evaluate_bag.py _plot_dir:="/plots" _bag:="/bag/buero.bag"