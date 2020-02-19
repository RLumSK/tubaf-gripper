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

import rosbag

from autonomy.PoseGenerator import *

import sys


# from: https://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console
def progress(count, total, suffix=''):
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    balken = '=' * filled_len + '-' * (bar_len - filled_len)
    absolut = str(count)+"/"+str(total)

    sys.stdout.write('[%s] %s%s [%s] ...%s\r' % (balken, percents, '%', absolut, suffix))
    sys.stdout.flush()  # As suggested by Rom Ruben


if __name__ == '__main__':
    rospy.init_node("evaluate_bag_set_pose", log_level=rospy.INFO)
    bag_path = rospy.get_param("~bag", '/home/grehl/bags/obj_cloud_2020-01-22/2020-01-22-13-16-05.bag')
    floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")
    obstacles_topic = rospy.get_param("~obstacles_topic", "/ork/tabletop/clusters")

    floor_msg = None
    obstacle_msg = None

    bag = rosbag.Bag(bag_path)
    n_msg = bag.get_message_count()
    i_msg = 0

    pca = PcaPoseGenerator("not_needed")
    dln = DelaunayPoseGenerator("not_needed")
    kde = MinimalDensityEstimatePoseGenerator("not_needed")
    mcr = MonteCarloPoseGenerator("not needed")
    lst_gen = [pca, dln, kde, mcr]

    evaluation = EvaluatePoseGenerators(lst_gen)
    formats = ['.tex', '.pdf']

    rospy.loginfo("Starting: bag has %g messages" % n_msg)
    for topic, msg, t in bag.read_messages(topics=[floor_topic, obstacles_topic]):
        i_msg += 1
        progress(i_msg-1, n_msg, suffix="of messages processed")
        if topic in obstacles_topic:
            rospy.logdebug("new obstacle_msg")
            obstacle_msg = msg
        elif topic in floor_topic:
            rospy.logdebug("new floor_msg")
            floor_msg = msg
        else:
            rospy.loginfo("unknown topic")
            continue

        if not pca.check_messages(obstacle_msg, floor_msg):
            rospy.logdebug("message don´t suit")
            continue

        evaluation.run(obs=obstacle_msg, flr=floor_msg)
        # if i_msg % 333 == 0:
        #     view_all(lst_generator=lst_gen, show_it=False, print_it=True, ff=formats)
        progress(i_msg, n_msg, suffix="of messages processed")

    evaluation.plot_heatmap(print_it=True, ff=formats)
    evaluation.distance_to(evaluation.dct_result[mcr.get_name()])
    print_plt(file_formats=formats)
    evaluation.evaluate(print_it=True, ff=formats)
