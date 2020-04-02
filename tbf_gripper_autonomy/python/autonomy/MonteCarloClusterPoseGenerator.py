#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, TU Bergakademie Freiberg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# author: grehl

import rospy

import multiprocessing as mp

import numpy as np
from scipy.spatial import Delaunay, KDTree
from sklearn.neighbors import NearestNeighbors

try:
    from autonomy.PoseGenerator import *
except ImportError as ie:
    import sys

    sys.path.append("/pkg/python/autonomy")
    from PoseGenerator import *

DF_N_CPU = mp.cpu_count()

class MonteCarloClusterPoseGenerator(PoseGeneratorRosInterface):

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS, mc_raster=DF_MC_RASTER, mc_wo=DF_MC_WO, n_cpu=DF_N_CPU):
        """
        Default constructor - Manage a Cluster of MonteCarloPoseGenerator in parallel, each one with a differnt position
        :param pub_topic: Topic where the calculated pose gets published
        :type pub_topic: str
        :param obs_topic: subscribing to obstacles messages from here
        :type obs_topic: str
        :param flr_topic: subscribing to floor messages from here
        :type flr_topic: str
        :param sub_sample: subsample ratio
        :type sub_sample: float
        :param enable_ros: whethere or not to use ROS
        :type enable_ros: bool
        :param mc_raster: Number of x and y line
        :type mc_raster: int
        :param mc_wo: weight for distance to obstacle
        :type mc_wo: float
        """
        super(MonteCarloClusterPoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)
        self.n_xlines = mc_raster
        self.n_ylines = self.n_xlines
        self.w = mc_wo

        self.n_cpu = n_cpu
        rospy.loginfo("[MonteCarloClusterPoseGenerator.__init__()] Using %d CPU's" % mp.cpu_count())

    def _generate(self, lst_obs_points, hull):
        """
        Parallel executable version of MonteCarloPoseGenerator._generate(...)
        :param lst_obs_points: all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: convex hull for the dataset
        :type hull: np.ndarray
        :return: tuple of the position [x, y, z]
        :rtype: np.ndarray
        """
        # see: https://stackoverflow.com/questions/30145957/plotting-2d-kernel-density-estimation-with-python
        xmin, xmax = min(hull[:, 0]), max(hull[:, 0])
        ymin, ymax = min(hull[:, 1]), max(hull[:, 1])

        # Generate data structure
        xx, yy = np.mgrid[xmin:xmax: self.n_xlines * 1j, ymin:ymax: self.n_ylines * 1j]
        all_positions = np.vstack([xx.ravel(), yy.ravel()]).T
        positions = PoseGeneratorRosInterface.in_hull(Delaunay(hull[:, :2], qhull_options="Pp"), all_positions)

        # See: https://www.machinelearningplus.com/python/parallel-processing-python/
        pool = mp.Pool(self.n_cpu)
        result_objects = pool.starmap_async(parallel_distances, [(i, p, lst_obs_points, hull, self.w) for i, p in enumerate(positions)]).get()
        pool.close()
        pool.join()

        i_max = np.argmax([r[3] for r in result_objects])
        return positions[i_max]


def parallel_distances(i, position, kd_obs, hull, w):
    d_obs = _obstacle_distance(kd_obs, position)
    d_hul = _hull_distance(hull, position)
    d_metric = _metric(d_obs, d_hul, w)
    return i, d_obs, d_hul, d_metric, position


def _metric(d_o, d_h, w):
    return PoseGeneratorRosInterface.metric(d_obstacles=d_o, d_hull=d_h, wo=w)


def _obstacle_distance(lst_obs_points, position):
    # return np.linalg.norm(position - kd_obstacles.query(position))
    obs_nn = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(lst_obs_points)
    d, _ = obs_nn.kneighbors([position[0:2]])
    return d[0]


def _hull_distance(hull, position):
    hull_distance, _ = PoseGeneratorRosInterface.calc_min_distance(position, hull.tolist(), mode="PL")
    return hull_distance
