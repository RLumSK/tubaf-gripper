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

from autonomy.PoseGenerator import *


if __name__ == '__main__':
    rospy.init_node("run_set_pose_generator", log_level=rospy.INFO)
    rospy.logdebug("{run_set_pose_generator.main()} starting")
    pub_topic = rospy.get_param("~pub_topic", "target_pose")

    pca = PcaPoseGenerator(pub_topic + "_pca")
    dln = DelaunayPoseGenerator(pub_topic + "_dln")
    kde = MinimalDensityEstimatePoseGenerator(pub_topic + "_kde")

    lst_generators = [pca, dln, kde]

    if rospy.get_param("~as_service", True):
        from autonomy.PoseGenerator import print_tex
        from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest, GenerateSetPoseResponse
        # for g in lst_generators:  # type: PoseGeneratorRosInterface
        #     g.unregister()

        def combined_service_request(request):
            max_nn = 0
            max_hl = 0
            i_nn = 0
            i_hl = 0
            lst_ps = []
            for i in range(0, len(lst_generators)):  # type: PoseGeneratorRosInterface
                generator = lst_generators[i]
                lst_ps.append(generator.once(obstacles_msg=request.obstacles, floor_msg=request.floor))
                ps = lst_ps[-1]

                generator.result = np.asarray([[ps.pose.position.x, ps.pose.position.y]])
                hull = generator.hull_points[:, :2]  # type: np.ndarray
                obstacles = generator.obs_points[:, :2]  # type: np.ndarray

                d_nn = EvaluatePoseGenerators.calc_min_distance(generator.result, obstacles.tolist(), mode="PP")
                if max_nn < d_nn:
                    max_nn = d_nn
                    i_nn = i
                d_hl = EvaluatePoseGenerators.calc_min_distance(generator.result, hull.tolist(), mode="PL")
                rospy.loginfo("{run_set_pose_generator.main().combined_service_request()}\n\thull distance: %s %g" %
                              (generator.get_name(), d_hl))
                if max_hl < d_hl:
                    max_hl = d_hl
                    i_hl = i
            if request.print_evaluation:
                print_tex(lst_generators)

            response = GenerateSetPoseResponse()
            if "nn" in request.policy or "nearest" in request.policy:
                response.set_pose = lst_ps[i_nn]
                gen = lst_generators[i_nn]
            elif "hl" in request.policy or "hull" in request.policy:
                response.set_pose = lst_ps[i_hl]
                gen = lst_generators[i_hl]
            else:
                rospy.logwarn("{run_set_pose_generator.main().combined_service_request()} unknown request.policy %s" %
                              request.policy)
                return response
            response.nn_distance = EvaluatePoseGenerators.calc_min_distance(
                gen.result, gen.obs_points[:, :2].tolist(), mode="PP")
            response.hull_distance = EvaluatePoseGenerators.calc_min_distance(
                gen.result, gen.hull_points[:, :2].tolist(), mode="PL")
            rospy.loginfo("{ServiceResponse} request.policy is %s choose result from: %s\n\t"
                          "max hull distance: %g %g\n\t"
                          "max nearest neighbor distance: %g %g" %
                          (request.policy, gen.get_name(),
                           max_hl, response.hull_distance,
                           max_nn, response.nn_distance))

            return response

        combined_service = rospy.Service('AllSetPoseGenerator_service', GenerateSetPose,  combined_service_request)
        rospy.spin()
    else:
        # Run in a loop
        _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
        _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")

        _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
        _floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

        while not rospy.is_shutdown():
            floor = None
            obstacles = None
            while not rospy.is_shutdown() and (floor is None or obstacles is None):
                floor = _floor_cache.getLast()
                obstacles = _obstacle_cache.getLast()
                rospy.sleep(1.0)
            for gen in lst_generators:  # type: PoseGeneratorRosView
                gen.once(obstacles_msg=obstacles, floor_msg=floor)
            view_all(lst_generators)
        rospy.spin()


    # _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
    # _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")
    #
    # _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
    # _floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)
    #
    # while True:
    #     try:
    #         request.floor = _floor_cache.getLast()
    #         request.obstacles = _obstacle_cache.getLast()
    #         if request.floor is None or request.obstacles is None:
    #             rospy.sleep(1.0)
    #             continue
    #         for service, name in zip([pca_service, dln_service, kde_service], [pca.__class__.__name__,
    #                                                                            dln.__class__.__name__,
    #                                                                            kde.__class__.__name__]):
    #             request.algorithm = name
    #             rospy.loginfo("[main] %s says %s" % (name, service(request)))
    #     except rospy.ServiceException as e:
    #         rospy.logerr("[main] Service %s call failed\n%s" % (name, e.message))

    # Evaluation
    # evaluation = EvaluatePoseGenerators([pca, dln, kde])
    # evaluation.perform(samples=rospy.get_param("~n_samples", 1000))

    # Test all


    # while not rospy.is_shutdown():
    #     a_lst = pca.evaluate()
    #     if a_lst is None:
    #         continue
    #     pca_h_min, pca_o_min = a_lst
    #     dln_h_min, dln_o_min = dln.evaluate()
    #     kde_h_min, kde_o_min = kde.evaluate()
    #
    #     i = np.argmin(pca_h_min, dln_h_min, kde_h_min)
    #     if i == 0:
    #         print("Minimal Hull distance - %s" % pca.get_name())
    #     elif i == 1:
    #         print("Minimal Hull distance - %s" % dln.get_name())
    #     else:
    #         print("Minimal Hull distance - %s" % kde.get_name())
    #
    #     i = np.argmin(pca_o_min, dln_o_min, kde_o_min)
    #     if i == 0:
    #         print("Minimal Obstacle distance - %s" % pca.get_name())
    #     elif i == 1:
    #         print("Minimal Obstacle distance - %s" % dln.get_name())
    #     else:
    #         print("Minimal Obstacle distance - %s" % kde.get_name())

    # plt.ion()
    # lst = [kde, dln, pca]
    # view = ViewPoseGenerators(lst)
    # r = rospy.Rate(1.0)
    # while not rospy.is_shutdown():
    #     now = rospy.Time.now()
    #     for g in lst:
    #         g.once()
    #     view.view()
    #     rospy.logdebug("{test_SetPoseGenerator.main()} sleeping")
    #     r.sleep()