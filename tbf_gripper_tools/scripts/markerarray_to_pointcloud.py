#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, TU Bergakademie Freiberg
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

import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray


class MarkerArrayConverter(object):
    """
    Class is used to transform a MarkerArray message into another Message type, eg pointcloud
    """

    def __init__(self):
        """
        Default Constructor - setup subscriber, publisher and load parameter
        """
        # parameter
        _markerarray_topic = rospy.get_param('~filtered_clusters_topic', "/ork/obj_clusters")
        _pointcloud_topic = rospy.get_param('~pointcloud_topic', "/ork/obj_clouds")
        # publisher
        self._pointcloud_publisher = rospy.Publisher(name=_pointcloud_topic, data_class=PointCloud2, queue_size=10)
        # subscriber
        rospy.Subscriber(name=_markerarray_topic, data_class=MarkerArray, callback=self._on_msg)

    def _on_msg(self, msg):
        """
        Receiving a new MarkerArray and transforming it and publishing the results
        :param msg: list of markers
        :type msg: visualization_msgs.msgs.MarkerArray
        :return: -
        :rtype: None
        """
        if len(msg.markers) < 1:
            return
        lst_points = []
        for marker in msg.markers:
            for point in marker.points:
                lst_points.append([point.x, point.y, point.z])
        pcl_msg = pc2.create_cloud_xyz32(msg.markers[0].header, lst_points)
        self._pointcloud_publisher.publish(pcl_msg)


if __name__ == '__main__':
    rospy.init_node("marker_array_converter")
    obj = MarkerArrayConverter()
    rospy.spin()
