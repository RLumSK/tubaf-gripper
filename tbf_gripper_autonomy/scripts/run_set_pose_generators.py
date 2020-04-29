#!/usr/bin/python
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

import autonomy.PoseGenerator as pg
import autonomy.EvaluatePoseGenerator as ev

from message_filters import Cache, Subscriber
from object_recognition_msgs.msg import TableArray
from visualization_msgs.msg import MarkerArray

if __name__ == '__main__':
    rospy.init_node("run_set_pose_generator", log_level=rospy.INFO)
    rospy.logdebug("{run_set_pose_generator.main()} starting")

    pub_topic = rospy.get_param("~pub_topic", pg.DF_PUB_TOPIC)
    floor_topic = rospy.get_param("~floor_topic", pg.DF_FLR_TOPIC)
    obstacles_topic = rospy.get_param("~obstacles_topic", pg.DF_OBS_TOPIC)
    sub_sample = rospy.get_param("~sub_sample", pg.DF_SUB_SAMPLE)
    n_bins = rospy.get_param("~n_bins", pg.DF_N_BINS)
    mc_raster = rospy.get_param("~mc_raster", pg.DF_MC_RASTER)
    plot_dir = rospy.get_param("~plot_dir", pg.DF_PLT_SAVE_DIR)
    mc_weight = rospy.get_param("~ref_wo", pg.DF_MC_WO)

    pca = pg.PcaPoseGenerator(pub_topic, obstacles_topic, floor_topic, sub_sample, True)
    dln = pg.DelaunayPoseGenerator(pub_topic, obstacles_topic, floor_topic, sub_sample, True)
    kde = pg.MinimalDensityEstimatePoseGenerator(pub_topic, obstacles_topic, floor_topic, sub_sample,True, n_bins)
    mcr = pg.MonteCarloPoseGenerator(pub_topic, obstacles_topic, floor_topic, sub_sample, True, mc_raster, mc_weight)
    lst_gen = [pca, dln, kde, mcr]

    plot_dir = rospy.get_param("~plot_dir", pg.DF_PLT_SAVE_DIR)
    evaluator = ev.EvaluatePoseGenerators(lst_gen, timeit=False, save_dir=plot_dir)
    _obstacle_topic = rospy.get_param("~obstacles_topic", pg.DF_OBS_TOPIC)
    _floor_topic = rospy.get_param("~floor_topic", pg.DF_FLR_TOPIC)

    _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
    _floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

    rospy.spin()
