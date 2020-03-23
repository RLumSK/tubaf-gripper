FROM ros:melodic-robot
LABEL maintainer="grehl" mail="Steve.Grehl@informatik.tu-freiberg.de" company="TU Bergakademie Freiberg"

## Install python modules
WORKDIR /tmp
RUN rm -rf /var/lib/apt/lists/* \
 && apt-get update \
 && apt-get install -y \
 	python3-pip \
 	python3-tk \
 	ros-melodic-object-recognition-msgs \
 	ros-melodic-visualization-msgs \
 	ros-melodic-geometry-msgs \
 	ros-melodic-message-filters \
 	texlive-xetex \
 	ghostscript\
 	dvipng \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --upgrade pip
COPY ./docker/requirements.txt /tmp
RUN pip3 install -r requirements.txt
RUN pip3 install numpy-quaternion
# We need latex for matplotlib latex export: https://matplotlib.org/3.1.1/tutorials/text/usetex.html

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
# nvidia-docker run --volume /raid/localshare/grehl/bag:/in/bag --volume /raid/localshare/grehl/plot:/out/plots -u $(id -u):$(id -g) grehl/set_pose_evaluation evaluate_bag.py -p /in/bag/buero.bag
