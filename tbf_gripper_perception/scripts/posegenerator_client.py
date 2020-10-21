#!/usr/bin/python
# coding=utf-8
# Software License Agreement (MIT License)
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
# Author: grehl
import rospy

import tf
from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest
from tbf_gripper_perception.srv import IdentifyFloor, IdentifyFloorRequest, IdentifyFloorResponse
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, Cache
from object_recognition_msgs.msg import TableArray
from visualization_msgs.msg import MarkerArray
from tf import TransformListener


def transform_ps(ps, target_frame, tf_listener=None):
    """
    :param ps: PoseStamped
    :param target_frame: str
    :param tf_listener: tf.TransformListener
    :return: ps in target_frame as PoseStamped
    """
    if tf_listener is None:
        tf_listener = tf.TransformListener(rospy.Duration.from_sec(15.0))
    while True:
        try:
            # https://answers.ros.org/question/188023/tf-lookup-would-require-extrapolation-into-the-past/
            tf_listener.waitForTransform(target_frame, ps.header.frame_id, rospy.Time(0), rospy.Duration(4))
            tmp_ps = PoseStamped()
            tmp_ps.header.frame_id = ps.header.frame_id
            tmp_ps.pose = ps.pose
            return tf_listener.transformPose(target_frame, ps=tmp_ps)
        except tf.ExtrapolationException as ee:
            rospy.logerr("transform_ps: ExtrapolationException %s" % ee.message)
            rospy.logerr("transform_ps: Target frame %s" % target_frame)
            rospy.logerr("transform_ps: Source header %s" % ps.header)
            return ps
        except tf.LookupException as le:
            rospy.logerr("EquipmentTask.check_set_equipment_pose(): LookupException %s" % le.message)
            rospy.logerr("transform_ps: Target frame %s" % target_frame)
            rospy.logerr("transform_ps: Source header %s" % ps.header)
            return ps


def sense(tf_listener=None, evaluation=None):
    """
    Sense for a suitable pose in a set of clusters in a plane
    :return: determined pose
    :rtype: PoseStamped
    """

    # Identify the floor
    floor_identify_service_name = rospy.get_param("~floor_identify_service", "/ork/identify_floor_plane")
    rospy.loginfo("[sense()] floor_identify_service_name: %s" % floor_identify_service_name)
    rospy.wait_for_service(floor_identify_service_name)
    rospy.loginfo("[sense()] %s available" % floor_identify_service_name)
    floor_identify_service = rospy.ServiceProxy(floor_identify_service_name, IdentifyFloor)
    identify_floor_req = IdentifyFloorRequest()
    identify_floor_res = IdentifyFloorResponse()
    while not rospy.is_shutdown():
        planes = rospy.wait_for_message(rospy.get_param('~tables_topic', "/ork/table_array"), TableArray)
        identify_floor_req.planes = planes
        identify_floor_res = floor_identify_service(identify_floor_req)
        if identify_floor_res.success:
            break
        else:
            rospy.logwarn("[sense()] Identify floor plane failed")
            rospy.sleep(2.0)

    # As client
    # use: PcaPoseGenerator, MinimalDensityEstimatePoseGenerator, DelaunayPoseGenerator
    pose_generation_service_name = rospy.get_param("~sense_service_name", "DelaunayPoseGenerator_service")
    rospy.logdebug("[sense()] pose_generation_service_name: %s" % pose_generation_service_name)
    rospy.wait_for_service(pose_generation_service_name)
    rospy.logdebug("[sense()] %s available" % pose_generation_service_name)
    pose_generation_service = rospy.ServiceProxy(pose_generation_service_name, GenerateSetPose)

    request = GenerateSetPoseRequest()
    request.header = Header()
    request.header.stamp = rospy.Time.now()
    request.print_evaluation = False
    request.policy = "hl"
    request.floor = identify_floor_res.floor

    _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
    _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)

    ps = None

    while ps is None:
        try:
            if _obstacle_cache.getLast() is None:
                rospy.sleep(1.0)
                continue
            request.obstacles = _obstacle_cache.getLast()
            # rospy.logdebug("[sense()] Request:\n%s" % request)
            reply = pose_generation_service(request)
            # rospy.loginfo("[sense()] %s suggests %s" % (pose_generation_service_name, reply))
            ps = reply.set_pose

        except rospy.ServiceException as e:
            rospy.logerr("[main] Service %s call failed\n%s" % (pose_generation_service_name, e.message))
            ps = None

    # Adjust z-coordinate to fit to ground
    ret_ps = transform_ps(ps, "base_footprint", tf_listener)

    if evaluation:
        evaluation.sense_result(identify_floor_req.planes, request.floor, request.obstacles, ret_ps)

    ret_ps.pose.position.z = 0
    return ret_ps


if __name__ == '__main__':
    rospy.init_node("PoseGenerators_client", log_level=rospy.DEBUG)

    t = rospy.get_param("~pub_ps_topic", "/debug_pose")
    pub = rospy.Publisher(t, PoseStamped, queue_size=1)
    rospy.sleep(1.0)
    while not rospy.is_shutdown():
        pub.publish(sense())
