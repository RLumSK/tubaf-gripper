#!/usr/bin/python
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
import numpy as np

from message_filters import Subscriber, Cache

from object_recognition_msgs.msg import TableArray, Table
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point

from matplotlib.mlab import PCA

import abc, six


@six.add_metaclass(abc.ABCMeta)
class PoseGeneratorRosInterface(object):
    """
    Abstract class to determine a valid pose on a floor plane for an object to set on.
    It manages the communication towards ROS.
    Any child class has to implement the specific method in '_generate()'.
    """

    def __init__(self):
        """
        Default constructor
        """
        _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
        _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")

        self._obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
        self._floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

        self.pub = rospy.Publisher(rospy.get_param("~pub_topic", "target_pose"), PoseStamped)

    def perform(self):
        """
        Main Loop - Reads messages and generates poses
        :return: -
        :rtype: -
        """
        while True:
            rospy.sleep(1.0)
            obstacles_msg = self._obstacle_cache.getLast()  # type: MarkerArray
            floor_msg = self._floor_cache.getLast()  # type: TableArray
            if obstacles_msg is None or floor_msg is None:
                continue
                rospy.logdebug("")
            ps = self._generate(obstacles_msg, floor_msg.tables[0])  # type: PoseStamped
            self.pub.publish(ps)

    def _point_on_plane(self, pnt, pln):
        """
        Calculate the projection of the point on the plane
        :param pnt: 3D point
        :type pnt: Point
        :param pln: plane
        :type pln: Table
        :return: 2D point on the plane
        :rtype: array
        """
        # TODO: correct implementation
        return [pnt.x, pnt.y]

    @abc.abstractmethod
    def _generate(self, obs_msg, flr_msg):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param obs_msg: cluster of obstacles
        :type obs_msg: MarkerArray
        :param flr_msg: floor
        :type flr_msg: Table
        :return: valid pose
        :rtype: PoseStamped
        """
        pass


class PcaPoseGenerator(PoseGeneratorRosInterface):

    def __init__(self):
        """
        Default constructor
        """
        super(PcaPoseGenerator, self).__init__()

    def _generate(self, obs_msg, flr_msg):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param obs_msg: cluster of obstacles
        :type obs_msg: MarkerArray
        :param flr_msg: floor
        :type flr_msg: Table
        :return: valid pose
        :rtype: PoseStamped
        """
        all_points = []
        for obstacle in obs_msg.markers:  # type: Marker
            for point in obstacle.points:  # type: Point
                # TODO: filter near on plane?
                all_points.append(self._point_on_plane(point, flr_msg))
        obj = PCA(np.asarray(all_points))

        pca_values = obj.Y
        pca_x = pca_values[0]
        pca_y = pca_values[1]

        def get_largest_gap(series):
            max_dist = 0.0
            gap = [0.0, 0.0]
            for i in range(1, len(series)):
                dist = np.abs(series[i - 1] - series[i])
                if dist > max_dist:
                    max_dist = dist
                    gap = [series[i - 1], series[i]]
            return gap

        def get_sample_in_gap(g):
            return g[0] + 0.5 * (g[1] - g[0])

        g = np.asarray([get_sample_in_gap(get_largest_gap(pca_x)), get_sample_in_gap(get_largest_gap(pca_y))])

        tmp = []
        for pnt in flr_msg.convex_hull:  # type: Point
            tmp.append(pnt.z)
        z = np.mean(tmp)
        p = np.matmul(g, np.linalg.inv(obj.Wt)).tolist()
        ps = PoseStamped()
        ps.header = flr_msg.header
        ps.pose.position.x = p[0]
        ps.pose.position.y = p[1]
        ps.pose.position.z = z
        ps.pose.orientation = flr_msg.pose.orientation
        return ps


if __name__ == '__main__':
    rospy.init_node("test_SetPoseGenerator", log_level=rospy.DEBUG)
    obj = PcaPoseGenerator()
    obj.perform()
    rospy.spin()
