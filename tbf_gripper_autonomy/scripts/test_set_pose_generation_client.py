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

import signal
import sys
import rospy

from message_filters import Subscriber, Cache
from std_msgs.msg import Header
from object_recognition_msgs.msg import TableArray
from visualization_msgs.msg import MarkerArray
from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest
from geometry_msgs.msg import PoseStamped


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    rospy.init_node("test_SetPoseGenerator_client", log_level=rospy.INFO)
    # As client
    # use: PcaPoseGenerator, MinimalDensityEstimatePoseGenerator, DelaunayPoseGenerator
    service_name = rospy.get_param("~service_name", "PcaPoseGenerator")
    rospy.wait_for_service(service_name + '_service')
    service = rospy.ServiceProxy(service_name + '_service', GenerateSetPose)

    request = GenerateSetPoseRequest()
    request.header = Header()
    request.header.stamp = rospy.Time.now()
    request.print_evaluation = False
    request.policy = "hl"

    _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
    _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")

    _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
    _floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

    pose_pub = rospy.Publisher("set_ssb_pose", PoseStamped, queue_size=10)

    while True:
        try:
            request.floor = _floor_cache.getLast()
            request.obstacles = _obstacle_cache.getLast()
            if request.floor is None or request.obstacles is None:
                rospy.sleep(1.0)
                continue
            rospy.logdebug("[main] Request:\n%s" % request)
            reply = service(request)
            rospy.loginfo("[main] %s says %s" % (service_name, reply))

            pose_pub.publish(reply.set_pose)

        except rospy.ServiceException as e:
            rospy.logerr("[main] Service %s call failed\n%s" % (service_name, e.message))

