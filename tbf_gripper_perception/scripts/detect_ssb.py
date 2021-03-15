#!/usr/bin/python
# coding=utf-8
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, TU Bergakademie Freiberg
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

from object_detector.srv import LocateInCloud, LocateInCloudRequest, LocateInCloudResponse
from sensor_msgs.msg import PointCloud2


if __name__ == '__main__':
    rospy.init_node("detect_ssb", log_level=rospy.DEBUG)
    while not rospy.is_shutdown():
        pcl_msg = rospy.wait_for_message(
            rospy.get_param("~cloud_topic", default="/gripper_d435/depth_registered/points"),
            PointCloud2, 10)
        if pcl_msg is None:
            continue
        service_name = rospy.get_param("~ssb_detection_service_name", default='locate_ssb_in_cloud')
        rospy.wait_for_service(service_name)
        locate_service = rospy.ServiceProxy(service_name, LocateInCloud)
        if locate_service is None:
            continue
        request = LocateInCloudRequest()
        request.cloud_msg = pcl_msg
        response = locate_service(request)  # type: LocateInCloudResponse
