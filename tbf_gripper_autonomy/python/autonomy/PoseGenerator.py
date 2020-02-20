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
from __builtin__ import staticmethod

import rospy
import numpy as np
import signal
import abc
import sys
import os
import time

from six import add_metaclass
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors
from scipy.ndimage import gaussian_filter

from message_filters import Subscriber, Cache
from object_recognition_msgs.msg import TableArray, Table
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point
from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest, GenerateSetPoseResponse

import matplotlib

matplotlib.use('Qt5Agg')
from pylab import *
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.image import NonUniformImage
from matplotlib import ticker

# see: https://stackoverflow.com/questions/55554352/import-of-matplotlib2tikz-results-in-syntaxerror-invalid-syntax
import matplotlib2tikz


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


DF_OBS_TOPIC = "/ork/tabletop/clusters"
DF_FLR_TOPIC = "/ork/floor_plane"
DF_PUB_TOPIC = "/pub_set_pose"
DF_SUB_SAMPLE = 100
DF_ENABLE_ROS = True
DF_N_BINS = 10
DF_MC_RASTER = 100

@add_metaclass(abc.ABCMeta)
class PoseGeneratorRosInterface:
    """
    Abstract class to determine a valid pose on a floor plane for an object to set on.
    It manages the communication towards ROS.
    Any child class has to implement the specific method in '_generate()'.
    """

    HULL_THRESHOLD = 6

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE, enable_ros=DF_ENABLE_ROS):
        """
        Default constructor
        :param pub_topic: Topic where the calculated pose gets published
        :type pub_topic: str
        :param obs_topic: subscribing to obstacles messages from here
        :type obs_topic: str
        :param flr_topic: subscribing to floor messages from here
        :type flr_topic: str
        :param sub_sample: subsample ratio
        :type sub_sample: float
        :param enable_ros: whether or not to use ROS
        :type enable_ros: bool
        """
        self.subsample = sub_sample
        self.ros = enable_ros
        if self.ros:
            self._obstacle_cache = Cache(Subscriber(obs_topic, MarkerArray), 1, allow_headerless=True)
            self._floor_cache = Cache(Subscriber(flr_topic, TableArray), 1)

            self.pub = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)

            self._service = rospy.Service(type(self).__name__ + '_service', GenerateSetPose,
                                          self.handle_service_request)

            self._dbg_pub_projection = rospy.Publisher(obs_topic + "_projected", MarkerArray, queue_size=1)

        self.result = np.asarray([0, 0, 0])  # type: np.ndarray
        self.obs_points = np.asarray([])  # type np.ndarray
        self.hull_points = np.asarray([])  # type: np.ndarray
        self.lines = []  # type: list

    def handle_service_request(self, request):
        """
        Handle a ROS Service request and return the response
        :param request: floor, obstacles and algorithm
        :type request: GenerateSetPoseRequest
        :return: pose, distances
        :rtype: GenerateSetPoseResponse
        """

        ps = self.once(obstacles_msg=request.obstacles, floor_msg=request.floor)

        self.result = np.asarray([[ps.pose.position.x, ps.pose.position.y]])
        hull = self.hull_points[:, :2]  # type: np.ndarray
        obstacles = self.obs_points[:, :2]  # type: np.ndarray

        response = GenerateSetPoseResponse()
        response.set_pose = ps
        response.nn_distance, _ = EvaluatePoseGenerators.calc_min_distance(self.result, obstacles.tolist(), mode="PP")
        response.hull_distance, _ = EvaluatePoseGenerators.calc_min_distance(self.result, hull.tolist(), mode="PP")

        if request.print_evaluation:
            print_tex(self)

        return response

    def get_name(self):
        """
        Returns the name
        :return: name
        :rtype: str
        """
        return str(self.__class__.__name__)[:-13]

    def perform(self):
        """
        Main Loop - Reads messages and generates poses
        :return: -
        :rtype: -
        """
        while True:
            rospy.logdebug("[PoseGeneratorRosInterface.perform] %s is alive" % self.get_name())
            self.pub.publish(self.once())

    def get_messages(self, current_time=None):
        """
        Get obstacles and floor message
        :param current_time: optional timestamp
        :type current_time: Time
        :return: tuple of messages
        :rtype: (MarkerArray, TableArray)
        """
        t = current_time
        if t is None:
            t = self._obstacle_cache.getLatestTime()
        obstacles_msg = self._obstacle_cache.getElemBeforeTime(t)  # type: MarkerArray
        floor_msg = self._floor_cache.getElemBeforeTime(t)  # type: TableArray
        return [obstacles_msg, floor_msg]

    def once(self, current_time=None, obstacles_msg=None, floor_msg=None):
        """
        Generate pose and publish it
        :param floor_msg: [optional] Floor
        :type floor_msg: TableArray
        :param obstacles_msg: [optional] Obstacles
        :type obstacles_msg: MarkerArray
        :param current_time: timestamp of the messages
        :type current_time: rospy.Time
        :return: pose
        :rtype: PoseStamped
        """
        t = current_time
        ps = PoseStamped()
        if self.ros:
            while t is None:
                t = self._obstacle_cache.getLatestTime()
            while obstacles_msg is None:
                obstacles_msg = self._obstacle_cache.getElemBeforeTime(t)  # type: MarkerArray
            while floor_msg is None:
                floor_msg = self._floor_cache.getElemBeforeTime(t)  # type: TableArray
            print self.ros
            ps.header = floor_msg.header
            ps.header.stamp = rospy.Time.now()
            ps.header.stamp = t

        if not self.check_messages(obstacles_msg, floor_msg):
            rospy.loginfo("[PoseGeneratorRosInterface.once()] messages not suitable\n%s\n---\n%s" %
                          (obstacles_msg, floor_msg))
            return ps

        self.obs_points, self.hull_points = self.extract_points(obstacles_msg, floor_msg.tables[0])
        if len(self.obs_points) == 0:
            # TODO: There are messages with obstacles but sometihng is strange with obstacles_msg.markers[0] when there are more then one
            return ps
        # rospy.loginfo("[PoseGeneratorRosInterface.once(%s)] #Obstacles: %g" % (self.get_name(), len(self.obs_points)))
        # rospy.loginfo("[PoseGeneratorRosInterface.once(%s)] Obstacles: %s" % (self.get_name(), str(self.obs_points)))
        # rospy.loginfo("[PoseGeneratorRosInterface.once(%s)] Obstacles: %s" % (self.get_name(), str(obstacles_msg)))
        valid_points = PoseGeneratorRosInterface.in_hull(Delaunay(self.hull_points[:, :2], qhull_options="Pp"),
                                                         self.obs_points[:, :2])  # type: np.ndarray
        if len(valid_points) == 0:
            rospy.logdebug("[PoseGeneratorRosInterface.once()] No obstacle points inside the hull")
            # rospy.sleep(0.1)  # next iteration most likely also without obstacles
            self.result = None
        else:
            self.result = self._generate(valid_points, hull=self.hull_points[:, :2])  # type: np.ndarray

        if self.result is None:
            self.result = np.asarray([floor_msg.tables[0].pose.position.x, floor_msg.tables[0].pose.position.y])

        ps = PoseGeneratorRosInterface._as_pose_stamped(self.result, floor_msg.tables[0])
        if len(self.result) == 0:
            self.result = np.hstack([self.result, ps.pose.position.x])
        if len(self.result) == 1:
            self.result = np.hstack([self.result, ps.pose.position.y])
        if len(self.result) == 2:
            self.result = np.hstack([self.result, ps.pose.position.z])
        return ps

    def publish_default(self, floor_message):
        """
        Publish the center of the floor as pose
        :param floor_message: floor
        :type floor_message: TableArray
        :return: pose
        :rtype: PoseStamped
        """
        ps = PoseStamped()
        ps.pose = floor_message.tables[0].pose
        if self.ros:
            ps.header = floor_message.header
            ps.header.stamp = rospy.Time.now()
            self.pub.publish(ps)
        return ps

    def check_messages(self, obs_msg, flr_msg):
        """
        Verifies if messages are suitable for further analysis
        :param obs_msg: Obstacles on the floor
        :type obs_msg: MarkerArray
        :param flr_msg: Floor plane
        :type flr_msg: TableArray
        :return: decision
        :rtype: bool
        """
        if obs_msg is None or flr_msg is None:
            rospy.logdebug("[PoseGeneratorRosInterface.check_messages()] One or both messages is/are None")
            return False
        if len(flr_msg.tables) == 0:
            rospy.logwarn("[PoseGeneratorRosInterface.check_messages()] No floor plane given")
            return False
        if len(flr_msg.tables[0].convex_hull) < PoseGeneratorRosInterface.HULL_THRESHOLD:
            rospy.logwarn("[PoseGeneratorRosInterface.check_messages()]  Linear hull of the floor is to small (%g/%g)"
                          % (len(flr_msg.tables[0].convex_hull), PoseGeneratorRosInterface.HULL_THRESHOLD))
            return False
        if len(obs_msg.markers) != 0:
            for marker in obs_msg.markers:  # type: Marker
                if len(marker.points) > 0:
                    return True
        rospy.logwarn("[PoseGeneratorRosInterface.check_messages()] No Obstacles given")
        self.publish_default(flr_msg)
        return False

    def unregister(self):
        """
        Skips the subscriptions and publish pipeline - useful if just used as a service
        :return: -
        :rtype: -
        """
        del self._obstacle_cache
        del self._floor_cache
        self._dbg_pub_projection.unregister()
        self.pub.unregister()
        del self._dbg_pub_projection
        del self.pub

    @staticmethod
    def calculate_plane_equation(floor_msg):
        """
        Calculate the plane equation: ax + by + cz + d = 0
        :param floor_msg: floor plane
        :type floor_msg: Table
        :return: [a, b, c, d]
        :rtype: list
        """
        from scipy.spatial.transform import Rotation

        def global_coords(flr_msg, i):
            x = flr_msg.convex_hull[i].x - flr_msg.pose.position.x
            y = flr_msg.convex_hull[i].y - flr_msg.pose.position.y
            z = flr_msg.convex_hull[i].z - flr_msg.pose.position.z
            rot = Rotation.from_quat([flr_msg.pose.orientation.x, flr_msg.pose.orientation.y,
                                      flr_msg.pose.orientation.z, flr_msg.pose.orientation.w])
            result = np.matmul(rot.as_dcm(), [x, y, z])  # type: np.ndarray
            return result.tolist()

        # see: http://kitchingroup.cheme.cmu.edu/blog/2015/01/18/Equation-of-a-plane-through-three-points/
        # Get plane equation: ax + by + cz + d = 0
        p0 = np.array(global_coords(floor_msg, 0))
        p1 = np.array(global_coords(floor_msg, 1))
        p2 = np.array(global_coords(floor_msg, 2))

        v1 = p1 - p0
        v2 = p2 - p0

        n = np.cross(v1, v2)
        a, b, c = n
        d = -np.dot(n, p2)
        return [a, b, c, d]

    @staticmethod
    def point_on_plane(pnt, pln):
        """
        Calculate the projection of the point on the plane
        :param pnt: 3D point
        :type pnt: Point
        :param pln: plane parameters for: ax + by + cz + d = 0
        :type pln: list
        :return: 2D point on the plane
        :rtype: array
        """
        a = pln[0]
        b = pln[1]
        c = pln[2]
        d = pln[3]
        # see: https://www.geeksforgeeks.org/find-the-foot-of-perpendicular-of-a-point-in-a-3-d-plane/
        x1 = pnt.x
        y1 = pnt.y
        z1 = pnt.z
        k = (-a * x1 - b * y1 - c * z1 - d) / (a * a + b * b + c * c)
        x2 = a * k + x1
        y2 = b * k + y1
        z2 = c * k + z1

        return [x2, y2, z2]

    @abc.abstractmethod
    def _generate(self, lst_obs_points, hull=None):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_obs_points: list with all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: optional give the linear hull included in lst_points
        :type hull: list
        :return: tuple of the position [x, y, z]
        :rtype: np.ndarray
        """
        pass

    def extract_points(self, obs_msg, flr_msg):
        """
        Project obstacles and convex hull on the floor plane
        :param obs_msg: cluster of obstacles
        :type obs_msg: MarkerArray
        :param flr_msg: floor
        :type flr_msg: Table
        :return: list with all points projected on the floor plane - [obstacles, hull]
        :rtype: [np.ndarray, np.ndarray]
        """
        start = 0
        if len(obs_msg.markers) > 1:
            start = 1
        # show the numbers in the obstacle message
        n_obstacles = 0
        rospy.logdebug("[PoseGeneratorRosInterface.extract_points(%s)] len(obs_msg.markers): %g" %
                      (self.get_name(), len(obs_msg.markers)))
        for obstacle in obs_msg.markers[start:]:  # type: Marker
            rospy.logdebug("[PoseGeneratorRosInterface.extract_points(%s)] len(obstacle.points): %g" %
                          (self.get_name(), len(obstacle.points)))
            n_obstacles += len(obstacle.points)
        rospy.logdebug("[PoseGeneratorRosInterface.extract_points(%s)] in total: %g" % (self.get_name(), n_obstacles))

        obstacles = []
        dbg_pnts = []
        dbg_markers = []
        plane_parameters = PoseGeneratorRosInterface.calculate_plane_equation(flr_msg)

        i = long(0)
        for obstacle in obs_msg.markers[start:]:  # type: Marker
            if self.subsample >= 1.0:
                ss = self.subsample
            else:
                ss = int(n_obstacles * self.subsample)
            if ss == 0:
                rospy.logwarn("[PoseGeneratorRosInterface.extract_points] Given %g points and a subsampling rate of %g "
                              "we use %g points - impossible, hence useing at least one" %
                              (len(obstacle.points), self.subsample, len(obstacle.points) * self.subsample))
                point = obstacle.points[len(obstacle.points) % 42]
                prjt_pnt = PoseGeneratorRosInterface.point_on_plane(point, plane_parameters)
                obstacles.append(prjt_pnt)
                point.x = prjt_pnt[0]
                point.y = prjt_pnt[1]
                point.z = prjt_pnt[2]
                dbg_pnts.append(point)
                continue
            for point in obstacle.points:  # type: Point
                i += 1
                if i % ss == 0:
                    prjt_pnt = PoseGeneratorRosInterface.point_on_plane(point, plane_parameters)
                    obstacles.append(prjt_pnt)
                    point.x = prjt_pnt[0]
                    point.y = prjt_pnt[1]
                    point.z = prjt_pnt[2]
                    dbg_pnts.append(point)

        rospy.logdebug("[PoseGeneratorRosInterface.extract_points(%s)] #Markers: %s" % (self.get_name(), len(obstacles)))
        # Add convex hull to dataset
        hull_pnts = []
        for i in range(0, len(flr_msg.convex_hull)):
            prjt_pnt = PoseGeneratorRosInterface.point_on_plane(flr_msg.convex_hull[i], plane_parameters)
            hull_pnts.append(prjt_pnt)
            a_point = Point()
            a_point.x = prjt_pnt[0]
            a_point.y = prjt_pnt[1]
            a_point.z = prjt_pnt[2]
            # dbg_pnts.append(a_point)

        if len(obs_msg.markers) > 0:
            dbg_markers.append(obs_msg.markers[0])
            obs_msg.markers[0].points = dbg_pnts
            if self.ros:
                self._dbg_pub_projection.publish(obs_msg)

        hull_pnts = np.asarray(hull_pnts)
        obstacles = np.asarray(obstacles)
        return [obstacles, hull_pnts]

    @staticmethod
    def _as_pose_stamped(lst_pos, floor_msg):
        # type: (np.ndarray, Table) -> PoseStamped
        """
        Generate the target pose for the given position by using the orientation of the plane
        :param lst_pos: position [x, y, z]
        :type lst_pos: np.ndarray
        :param floor_msg: floor plane
        :type floor_msg: Table
        :return: target pose
        :rtype: PoseStamped
        """
        if isinstance(lst_pos, np.ndarray):
            lst_pos = lst_pos.tolist()
        if len(lst_pos) == 0:
            lst_pos.append(floor_msg.pose.position.x)
        if len(lst_pos) == 1:
            lst_pos.append(floor_msg.pose.position.y)
        if len(lst_pos) == 2:
            lst_pos.append(floor_msg.pose.position.z)
        ps = PoseStamped()
        ps.header = floor_msg.header
        ps.pose.position.x = lst_pos[0]
        ps.pose.position.y = lst_pos[1]
        ps.pose.position.z = lst_pos[2]
        ps.pose.orientation = floor_msg.pose.orientation
        # rospy.logdebug("[PcaPoseGenerator._generate()] ps.pose: %s" % ps.pose)
        return ps

    @staticmethod
    def in_hull(hull, pnts):
        """
        Determine all points within the given hull
        :param hull: linear hull
        :type hull: Delaunay or list
        :param pnts: points
        :type pnts: np.ndarray or list
        :return: list with points inside of the hull
        :rtype: np.ndarray
        """
        # From:
        # https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
        if not isinstance(hull, Delaunay):
            lin_hull = Delaunay(np.asarray(hull)[:, :2], qhull_options="Pp")
        else:
            lin_hull = hull
        valid_sample = []
        for s in pnts:
            if lin_hull.find_simplex(s) >= 0:
                valid_sample.append(s)
        return np.asarray(valid_sample)

    @staticmethod
    def metric(d_obstacles, d_hull):
        """
        Given a distance to the obstacle and a distance to the convex hull, the combined distance is calculated
        :param d_obstacles: distance to the nearest obstacle
        :type d_obstacles: float
        :param d_hull: distance to the convex hull, perpendicular to the nearest line
        :type d_hull: float
        :return: combined distance
        :rtype: float
        """
        ret_val = float(d_obstacles) * float(d_hull)
        rospy.logdebug("[PoseGeneratorRosInterface.metric()] %g *%g = %g" % (d_obstacles, d_hull, ret_val))
        return ret_val


@add_metaclass(abc.ABCMeta)
class PoseGeneratorRosView(PoseGeneratorRosInterface):
    """
    Implement a matplotlib interface for the generic PoseGeneratorRosInterface class - still abstract tho
    """

    @staticmethod
    def get_color(name):
        """
        Return a color identifier for the given name
        See: https://matplotlib.org/2.0.2/api/colors_api.html
        :param name: custom name, eg DelaunayPoseGenerator
        :type name: str
        :return: color_id
        :rtype: char
        """
        if "stacle" in name or "indernis" in name:
            return 'C0'
        elif "roundarea" in name or "oden" in name:
            return 'C1'
        elif "elaunay" in name:
            return 'C2'
        elif "inimalDensityEstimate" in name:
            return 'blue'
        elif "onteCarlo" in name:
            return 'k'
        elif "ca" in name:
            return 'C4'
        else:
            return 'C6'

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS):
        """
        Default constructor
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
        """
        super(PoseGeneratorRosView, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)

    def plot(self, ax=None, hull=True, obstacles=True):
        """
        Plot the information of this PoseGenerator into an outside existing matplotlib figure
        :param hull: plot the hull
        :type hull: bool
        :param obstacles: plot the obstacles
        :type obstacles: bool
        :param ax: axis handle
        :type ax: plt.axes.Axis
        :return: data point axis
        :rtype: plt.axes.Axis
        """
        if ax is None:
            fig = plt.figure()
            ax = fig.gca()
        n = self.get_name()
        c = PoseGeneratorRosView.get_color(n)
        if len(self.result.shape) == 1 and self.result.shape[0] == 3:
            self.result = np.asarray([self.result])
        for lne in self.lines:
            ax.plot([lne[0][0], lne[1][0]], [lne[0][1], lne[1][1]], ':', color=c, alpha=0.75, zorder=5,
                    label=n + " Hilfslinien")

        if obstacles:
            as_array = np.asarray(self.obs_points)
            n = "Hindernisse"
            rospy.loginfo("[PoseGeneratorRosView.plot(%s)] obstacles: %s" % (n, as_array))
            ax.plot(as_array[:, 0], as_array[:, 1], '.', label=n, zorder=3.5, color=PoseGeneratorRosView.get_color(n))
        if hull:
            hull = self.hull_points.tolist()
            hull.append(hull[0])
            as_array = np.asarray(hull)
            n = "Boden"
            ax.plot(as_array[:, 0], as_array[:, 1], '+--', label=n, zorder=4.5, color=PoseGeneratorRosView.get_color(n))
        return ax.plot(self.result[0, 0], self.result[0, 1], 'o', ms=12, zorder=100, color=c, label=n)


class PcaPoseGenerator(PoseGeneratorRosView):

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS):
        """
        Default constructor
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
        """
        super(PcaPoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)

    def _generate(self, lst_obs_points, hull=None):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_obs_points: list with all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: optional give the linear hull included in lst_points
        :type hull: list
        :return: tuple of the position [x, y, z] or None
        :rtype: np.ndarray
        """
        if lst_obs_points is None or len(lst_obs_points) == 0:
            rospy.logwarn("[PcaPoseGenerator._generate()] No obstacle points given")
            return None
        if hull is None:
            hull = []
        rospy.logdebug("[PcaPoseGenerator._generate()] Starting")

        if len(lst_obs_points) < 3:
            rospy.logdebug("[PcaPoseGenerator._generate()] Not enough obstacle points: %g" % len(lst_obs_points))
            rospy.logdebug("[PcaPoseGenerator._generate()] %s" % lst_obs_points)
            return None

        # see: https://scikit-learn.org/stable/auto_examples/decomposition/plot_pca_iris.html
        from sklearn import decomposition
        obs2d_pca = decomposition.PCA(n_components=2).fit(lst_obs_points)  # type: decomposition.PCA
        pca_values = obs2d_pca.transform(lst_obs_points)  # type: np.ndarray
        hull_values = obs2d_pca.transform(hull)  # type: np.ndarray
        pca_x = np.append(pca_values[:, 0], [np.min(hull_values[:, 0]), np.max(hull_values[:, 0])])
        pca_y = np.append(pca_values[:, 1], [np.min(hull_values[:, 1]), np.max(hull_values[:, 1])])

        def get_sample_in_gap(_g):
            return _g[0] + 0.5 * (_g[1] - _g[0])

        def get_largest_gap(series, blacklst=None):
            if blacklst is None:
                blacklst = []
            series.sort()
            max_dist = 0.0
            gap = [0.0, 0.0]
            for i in range(1, len(series)):
                if series[i] in blacklst and series[i - 1] in blacklst:
                    continue
                distance = np.abs(series[i] - series[i - 1])
                if distance > max_dist:
                    max_dist = distance
                    gap = [series[i - 1], series[i]]
            # rospy.logdebug("[PcaPoseGenerator._generate().get_largest_gap()] max_dist: %g\t gap: %s" % (max_dist, gap))
            return gap

        limits = [get_largest_gap(pca_x), get_largest_gap(pca_y)]
        g_sample = [get_sample_in_gap(limits[0]), get_sample_in_gap(limits[1])]
        h = Delaunay(np.asarray(hull_values), qhull_options="Pp")
        while len(PoseGeneratorRosInterface.in_hull(h, [g_sample])) == 0:
            new_sample = [get_sample_in_gap(get_largest_gap(pca_x, blacklst=limits[0])), g_sample[1]]
            if len(PoseGeneratorRosInterface.in_hull(h, [new_sample])) != 0:
                g_sample = new_sample
                break
            new_sample = [g_sample[0], get_sample_in_gap(get_largest_gap(pca_y, blacklst=limits[1]))]
            if len(PoseGeneratorRosInterface.in_hull(h, [new_sample])) != 0:
                g_sample = new_sample
                break
            new_limits = [get_largest_gap(pca_x, blacklst=limits[0]), get_largest_gap(pca_y, blacklst=limits[1])]
            limits[0].extend(new_limits[0])
            limits[1].extend(new_limits[1])
            g_sample = [get_sample_in_gap(new_limits[0]), get_sample_in_gap(new_limits[1])]

        p = obs2d_pca.inverse_transform(g_sample)
        rospy.logdebug("[PcaPoseGenerator._generate()] p: %s  in hull? %s" %
                       (p[:2], len(PoseGeneratorRosInterface.in_hull(hull, [p[:2]])) > 0))
        # Save PCA axis as lines
        x0 = obs2d_pca.inverse_transform([min(pca_x), 0])
        x1 = obs2d_pca.inverse_transform([max(pca_x), 0])
        y0 = obs2d_pca.inverse_transform([0, min(pca_y)])
        y1 = obs2d_pca.inverse_transform([0, max(pca_y)])
        self.lines = [[x0, x1], [y0, y1]]
        return np.asarray(p)


class DelaunayPoseGenerator(PoseGeneratorRosView):

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS):
        """
        Default constructor
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
        """
        super(DelaunayPoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)

    def _generate(self, lst_obs_points, hull=None):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_obs_points: list with all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: optional give the linear hull included in lst_points
        :type hull: list
        :return: tuple of the position [x, y, z]
        :rtype: np.ndarray
        """
        if hull is None:
            hull = []
        rospy.logdebug("[DelaunayPoseGenerator._generate()] Starting")
        hull = np.asarray(hull)
        rospy.logdebug("[DelaunayPoseGenerator._generate()] Shape - obs: %s\thull: %s" %
                       (lst_obs_points.shape, hull.shape))
        all_points = np.vstack((lst_obs_points, hull))
        points = np.asarray(all_points)

        _delaunay = Delaunay(points[:, :2], incremental=False, qhull_options="Pp")  # type: Delaunay

        def distance(_p1, _p2):
            d = _p2 - _p1
            return (d[0] ** 2 + d[1] ** 2) ** 0.5

        def area(pnts):
            assert len(pnts) == 3
            i_a = distance(pnts[0], pnts[1])
            i_b = distance(pnts[0], pnts[2])
            i_c = distance(pnts[1], pnts[2])
            # formula of Heron: https://en.wikipedia.org/wiki/Heron%27s_formula#Numerical_stability
            a = max(i_a, i_b, i_c)
            b = np.median([i_a, i_b, i_c])
            c = min(i_a, i_b, i_c)
            return 0.25 * ((a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c))) ** 0.5

        max_distance = 0
        max_idx = [0, 0, 0]
        for triangle in _delaunay.simplices:  # type: list
            p0 = _delaunay.points[triangle[0]]
            p1 = _delaunay.points[triangle[1]]
            p2 = _delaunay.points[triangle[2]]
            v = area([p0, p1, p2])
            if max_distance < v:
                max_distance = v
                max_idx = [triangle[0], triangle[1], triangle[2]]
                self.lines = [[p0, p1], [p1, p2], [p2, p0]]
                rospy.logdebug("[DelaunayPoseGenerator._generate()] max_area: %g" % max_distance)

        def get_center_point(pnts):
            assert len(pnts) == 3
            pnts = np.asarray(pnts)
            # see: https://de.wikipedia.org/wiki/Inkreis#Koordinaten
            a = distance(pnts[1], pnts[2])
            b = distance(pnts[0], pnts[2])
            c = distance(pnts[0], pnts[1])
            p = a + b + c
            x = (a * pnts[0, 0] + b * pnts[1, 0] + c * pnts[2, 0]
                 ) / p
            y = (a * pnts[0, 1] + b * pnts[1, 1] + c * pnts[2, 1]
                 ) / p
            return [x, y]

        return np.asarray(get_center_point([_delaunay.points[max_idx[0]],
                                            _delaunay.points[max_idx[1]],
                                            _delaunay.points[max_idx[2]]]))


class MinimalDensityEstimatePoseGenerator(PoseGeneratorRosView):
    """
    Determine the Pose using KDE and discretization
    """

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS, n_bins=DF_N_BINS):
        """
        Default constructor
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
        :param n_bins: number of points of the grid per dimension
        :typen_bins: int
        """
        super(MinimalDensityEstimatePoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)
        self.n_bins = n_bins
        # self.hlp_xx, self.hlp_yy = np.mgrid[-1:1: self.n_bins * 1j, -1:1: self.n_bins * 1j]
        self.hlp_positions = np.zeros((self.n_bins ** 2, 2))
        self.hlp_f = np.zeros(self.n_bins ** 2)

    def _generate(self, lst_obs_points, hull=None):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_obs_points: list with all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: optional give the linear hull included in lst_points
        :type hull: list
        :return: tuple of the position [x, y, z]
        :rtype: np.ndarray
        """
        # see: https://stackoverflow.com/questions/41577705/how-does-2d-kernel-density-estimation-in-python-sklearn-work
        if hull is None:
            hull = []
        hull = np.asarray(hull)

        # see: https://stackoverflow.com/questions/30145957/plotting-2d-kernel-density-estimation-with-python
        import scipy.stats as st
        x = np.concatenate((lst_obs_points[:, 0], hull[:, 0]))
        y = np.concatenate((lst_obs_points[:, 1], hull[:, 1]))
        xmin, xmax = min(hull[:, 0]), max(hull[:, 0])
        ymin, ymax = min(hull[:, 1]), max(hull[:, 1])

        # Peform the kernel density estimate
        xx, yy = np.mgrid[xmin:xmax: self.n_bins * 1j, ymin:ymax: self.n_bins * 1j]
        all_positions = np.vstack([xx.ravel(), yy.ravel()]).T
        positions = PoseGeneratorRosInterface.in_hull(Delaunay(hull[:, :2], qhull_options="Pp"), all_positions)
        values = np.vstack([x, y])
        kernel = st.gaussian_kde(values)
        # print all_positions.shape  # n_bins **2 x 2
        # print positions.shape  # i x 2, i < n_bins**2
        # print xx.shape # n_bins x n_bins
        self.hlp_f = kernel(all_positions.T)
        self.hlp_positions = all_positions
        z_i = np.argmin(kernel(positions.T))
        z_x = positions[z_i, 0]
        z_y = positions[z_i, 1]

        return np.asarray([z_x, z_y])

    def plot(self, ax, hull=True, obstacles=True):
        """
        Plot the information of this PoseGenerator into an outside existing matplotlib figure
        :param hull: plot the hull
        :type hull: bool
        :param obstacles: plot the obstacles
        :type obstacles: bool
        :param ax: axis handle
        :type ax: plt.axes.Axis
        :return: data point axis
        :rtype: plt.axes.Axis
        """
        ret_ax = super(MinimalDensityEstimatePoseGenerator, self).plot(ax, hull, obstacles)
        l = int(np.sqrt(len(self.hlp_f)))
        xmin, xmax = min(self.hlp_positions[:, 0]), max(self.hlp_positions[:, 0])
        ymin, ymax = min(self.hlp_positions[:, 1]), max(self.hlp_positions[:, 1])
        xx, yy = np.mgrid[xmin:xmax:l * 1j, ymin:ymax:l * 1j]
        f = np.reshape(self.hlp_f[:l ** 2].T, xx.shape)
        cs = ax.contourf(xx, yy, f, cmap='Blues', alpha=0.5, zorder=1, locator=ticker.LogLocator())
        cbar = plt.gcf().colorbar(cs)
        cset = ax.contour(xx, yy, f, colors=PoseGeneratorRosView.get_color(self.get_name()), alpha=0.2, zorder=2)
        ax.clabel(cset, inline=True, fontsize=10, fmt='%1.2f', zorder=3)

        return ret_ax


class MonteCarloPoseGenerator(PoseGeneratorRosView):
    """
    Discretion of the search area and run nn approach for hull and obstacles
    """

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS, mc_raster=DF_MC_RASTER):
        """
        Default constructor
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
        """
        super(MonteCarloPoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)
        self.n_xlines = mc_raster
        self.n_ylines = self.n_xlines

    def _generate(self, lst_obs_points, hull=None):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_obs_points: list with all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: optional give the linear hull included in lst_points
        :type hull: list
        :return: tuple of the position [x, y, z]
        :rtype: np.ndarray
        """
        # see: https://stackoverflow.com/questions/41577705/how-does-2d-kernel-density-estimation-in-python-sklearn-work
        if hull is None:
            hull = []
        hull = np.asarray(hull)

        # see: https://stackoverflow.com/questions/30145957/plotting-2d-kernel-density-estimation-with-python
        x = np.concatenate((lst_obs_points[:, 0], hull[:, 0]))
        y = np.concatenate((lst_obs_points[:, 1], hull[:, 1]))
        xmin, xmax = min(hull[:, 0]), max(hull[:, 0])
        ymin, ymax = min(hull[:, 1]), max(hull[:, 1])

        # Generate data structure
        xx, yy = np.mgrid[xmin:xmax: self.n_xlines * 1j, ymin:ymax: self.n_ylines * 1j]
        all_positions = np.vstack([xx.ravel(), yy.ravel()]).T
        positions = PoseGeneratorRosInterface.in_hull(Delaunay(hull[:, :2], qhull_options="Pp"), all_positions)

        # Distance to obstacles
        # see: https://scikit-learn.org/stable/modules/neighbors.html
        obs_nn = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(
            lst_obs_points)  # type: NearestNeighbors
        hul_nn = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(
            hull)  # type: NearestNeighbors
        distances, indices = obs_nn.kneighbors(positions)
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] distances = %s" % distances)
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] indices = %s" % indices)

        # Distance calculation
        extended_positions = np.ndarray((len(positions), 3))
        for i in range(0, len(positions)):
            hull_distance, _ = EvaluatePoseGenerators.calc_min_distance(positions[i], hull.tolist(), mode="PL")
            d, i_dp = obs_nn.kneighbors([positions[i, 0:2]])
            m = PoseGeneratorRosInterface.metric(d_obstacles=d[0], d_hull=hull_distance)
            extended_positions[i] = np.append(positions[i], m)
        i_max = np.argmax(extended_positions[:, -1])
        ret_pos = extended_positions[i_max, 0:-1]
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] position %s at index %g with distance %g" %
                       (ret_pos, i_max, extended_positions[i_max, -1]))
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] distances: \n%s" % extended_positions[:, -1])
        # Calculate help lines
        obs_d, obs_i = obs_nn.kneighbors([ret_pos])
        hul_d, hp = EvaluatePoseGenerators.calc_min_distance(ret_pos, hull.tolist(), mode="PL")
        obs_i = obs_i[0][0]
        obs = lst_obs_points[obs_i]

        self.lines = [[hp, ret_pos], [ret_pos, obs]]

        return ret_pos


def view_all(lst_generator, show_it=True, print_it=False, ff=['.tex', '.pdf']):
    """
    Plot all given generators into one figure
    :param ff:  [optional] list of file formats, eg pgf, pdf
    :type ff: list
    :param print_it: [optional] print the plot
    :type print_it: bool
    :param show_it: [optional] show the plot
    :type show_it: bool
    :param lst_generator: list of pose generators of type PoseGeneratorRosView
    :type lst_generator: list()
    :return: -
    :rtype:-
    """
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_xlim(-1, 1)
    ax.set_ylim(*ax.get_xlim())
    for generator in lst_generator:  # type: PoseGeneratorRosView
        if len(generator.obs_points) == 0:
            return
        generator.plot(ax, hull=False, obstacles=False)
        rospy.loginfo("[view_all(%s)] #obstacles: %g" % (generator.get_name(), len(generator.obs_points)))

    as_array = np.asarray(lst_generator[-1].obs_points)
    rospy.logdebug("[view_all()] #obstacles: %g" % len(lst_generator[-1].obs_points))
    name = "Hindernisse"
    ax.plot(as_array[:, 0], as_array[:, 1], '.', label=name, zorder=10, color=PoseGeneratorRosView.get_color(name))
    hull = lst_generator[-1].hull_points.tolist()
    hull.append(hull[0])
    as_array = np.asarray(hull)
    name = "Boden"
    ax.plot(as_array[:, 0], as_array[:, 1], '+--', label=name, zorder=9, color=PoseGeneratorRosView.get_color(name))

    # Text
    if lst_generator[-1].subsample > 1.0:
        txt = fig.text(0.15, 0.01, "Teilstichprobe alle " + str(lst_generator[-1].subsample) + " Hindernisse")
    else:
        txt = fig.text(0.15, 0.01,
                       "Teilstichprobe mit " + str(lst_generator[-1].subsample * 100.0) + "% der Hindernisse")

    def legend_without_duplicate_labels(axis):
        handles, labels = axis.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        # Place a legend above this subplot, expanding itself to
        # fully use the given bounding box.
        return axis.legend(*zip(*unique), bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=2,
                           mode="expand", borderaxespad=0.)

    lgd = legend_without_duplicate_labels(ax)

    if show_it:
        plt.show()
    if print_it:
        print_plt(file_formats=ff, suffix=u"Errechnete Absetzpunkte")


def print_plt(file_formats=['.pgf', '.pdf'], extras=[], save_dir="/home/grehl/Schreibtisch/PoseGeneratorImages",
              suffix=''):
    """
    Print the current figure to a file
    :param file_formats:  [optional] list of file formats, eg pgf, pdf
    :type file_formats: list
    :param extras: [optional] passed to matplotlib.savefig(bbox_extra_artists)
    :type extras: list
    :param save_dir: [optional] target directory
    :type save_dir: str
    :param suffix: [optional] file name suffix
    :type suffix: str
    :return: -
    :rtype: -
    """
    if not os.path.exists(save_dir):
        rospy.logwarn("[print_plt] Creating '%s' to store plots" % save_dir)
        os.makedirs(save_dir)
    p = os.path.join(save_dir, str(rospy.Time.now().to_nsec()) + suffix)
    for c in file_formats:
        if 'tex' in c or 'tikz' in c:
            matplotlib2tikz.save(p + c)
        else:
            plt.savefig(p + c, bbox_extra_artists=extras, bbox_inches='tight')
    plt.close()


def print_tex(generator, save_dir="/home/grehl/Schreibtisch/PoseGeneratorImages"):
    """
    Save the current state in a LaTeX conform plot
    :param generator: pose generator after at least one computation, or alist of them
    :type generator: PoseGeneratorRosView or list
    :param save_dir: directory where the plots are stored
    :type save_dir: str
    :return: -
    :rtype: -
    """
    view_all(generator, show_it=False)
    print_plt(file_formats=[".pdf", ".tex"], save_dir=save_dir)


class EvaluatePoseGenerators(object):
    """
    Evaluation for a PoseGeneratorRosInterface instance
    """

    C_MAP = u'tab20c_r'

    @staticmethod
    def calc_min_distance(point, lst_points, mode="PP"):
        """
        Given a point, calculate the minimal distance to a list of points
        :param point: point of interest
        :type point: np.ndarray
        :param lst_points: list of points to test against
        :type lst_points: list
        :param type: PP - point to point distance, PL - point to line distance
        :type mode: str
        :return: minimal distance, point of the perpendicular
        :rtype: float, np.ndarray
        """
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] point = %s" % point)
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] len(lst_points) = %s" % len(lst_points))
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] lst_points = %s" % lst_points)
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] mode = %s" % mode)
        nn = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(
            lst_points)  # type: NearestNeighbors
        tmp = nn.kneighbors([point])
        distance, i_min = tmp[0][0][0], tmp[1][0][0]
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] distance: %s" % distance)
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] i_min: %s" % i_min)
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] nn: %s" % lst_points[i_min])
        if "PP" in mode:
            return distance, lst_points[i_min]
        elif "PL" in mode:
            min_d = float('inf')
            min_f = np.asarray([0, 0])
            p1 = point[0:2]
            p3 = np.asarray(lst_points[-1])[0:2]
            for i in range(0, len(lst_points)):
                p2 = p3
                p3 = np.asarray(lst_points[i])[0:2]
                # d = np.cross(p2 - p1, p1 - p3) / np.linalg.norm(p2 - p1)
                f = EvaluatePoseGenerators.calc_perpendicular_point(p1, np.asarray([p2, p3]))
                # rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance(PL)] distance: %s" % d)
                # rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance(PL)] f: %s" % f)
                # rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance(PL)] |pf|: %s" % np.linalg.norm(f-p1))
                d = np.linalg.norm(f - p1)

                if d < min_d:
                    min_d = d
                    min_f = f  # perpendicular point
            rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance(PL)] distance: %g" % min_d)
            return min_d, min_f
        else:
            rospy.logwarn("[EvaluatePoseGenerators.calc_min_distance()] unknown mode %s" % mode)
            return float('NaN')

    @staticmethod
    def calc_perpendicular_point(p0, line):
        """
        Calculate the perpendicular of a given point on a given line
        :param p0: point not in line
        :type p0: np.ndarray
        :param line: line given by a set of two points in the line
        :type line: np.ndarray
        :return: perpendicular point in line
        :rtype: np.ndarray
        """
        # https://www.mathematik-oberstufe.de/vektoren/a/abstand-punkt-gerade-lot.html
        ax, ay = line[0, 0], line[0, 1]
        bx, by = line[1, 0] - line[0, 0], line[1, 1] - line[0, 1]
        rx, ry = p0[0], p0[1]
        s = (bx * (rx - ax) + by * (ry - ay)) / (bx ** 2 + by ** 2)
        f = np.asarray([ax, ay]) + s * np.asarray([bx, by])  # s in g
        return f

    @staticmethod
    def get_ident(obj):
        return str(type(obj)).split(".")[-1][:-2]

    def __init__(self, generators, timeit=True):
        """
        Default constructor
        :param generators: pose generators
        :type generators: list of PoseGeneratorRosInterface
        :param timeit: measure time during run
        :type timeit: bool
        """
        self._generators = generators

        self.dct_result = {}
        self.dct_lst_hull_distance = {}
        self.dct_lst_obstacle_distance = {}
        self.dct_count_largest_hull_distance = {}
        self.dct_count_largest_obstacle_distance = {}
        self.dct_timing = {}

        self.timeit = timeit

        for g in self._generators:  # type: PoseGeneratorRosInterface
            ident = g.get_name()
            self.dct_lst_hull_distance[ident] = []
            self.dct_lst_obstacle_distance[ident] = []
            self.dct_count_largest_hull_distance[ident] = 0
            self.dct_count_largest_obstacle_distance[ident] = 0
            self.dct_timing[ident] = []
            self.dct_result[ident] = []

    def perform(self, timeout=1.0, samples=1000):
        """
        Evaluate the generator for samples
        :param samples: number of iterations
        :type samples: int
        :param timeout: time between triggers
        :type timeout: float
        :return: -
        :rtype: -
        """
        i = 1
        n_samples = samples
        while not rospy.is_shutdown() and i <= n_samples:
            self.run()
            rospy.loginfo("[EvaluatePoseGenerators.perform()] %g/%g" % (i, n_samples))
            rospy.sleep(timeout)
            i += 1
        self.evaluate()

    def run(self, obs=None, flr=None):
        """
        Trigger given generators and save evaluation parameters
        :param obs: [optional] use the given obstacles
        :type obs: MarkerArray
        :param flr: [optional] use the given floor plane
        :type flr: TableArray
        :return: -
        :rtype: -
        """
        while not self._generators[0].check_messages(obs, flr):
            rospy.logdebug("[EvaluatePoseGenerators.run()] Getting new messages")
            obs, flr = self._generators[0].get_messages()
            rospy.sleep(1.0)

        d_hull = 0
        d_obst = d_hull
        hull_ident = ""
        obst_ident = hull_ident

        for g in self._generators:  # type: PoseGeneratorRosInterface
            ident = g.get_name()
            if self.timeit:
                t = time.time()
            g.once(obstacles_msg=obs, floor_msg=flr, current_time=flr.header.stamp)
            if self.timeit:
                self.dct_timing[ident].append(time.time() - t)
            result = np.asarray(g.result)
            self.dct_result[ident].append(result)
            obstacles = g.obs_points
            hull = g.hull_points
            if len(hull) == 0 or len(obstacles) == 0:
                return
            # try:
            hull_distance, _ = EvaluatePoseGenerators.calc_min_distance(result, hull, mode="PL")
            obstacle_distance, _ = EvaluatePoseGenerators.calc_min_distance(result, obstacles, mode="PP")
            self.dct_lst_hull_distance[ident].append(hull_distance)
            self.dct_lst_obstacle_distance[ident].append(obstacle_distance)
            rospy.logdebug("[EvaluatePoseGenerators.run() - %s] Min(Distance2Hull) = %g" %
                           (ident, hull_distance))
            rospy.logdebug("[EvaluatePoseGenerators.run() - %s] Min(Distance2Obstacles) = %g" %
                           (ident, obstacle_distance))
            # except ValueError as ex:
            #     rospy.logwarn("[EvaluatePoseGenerators.run() - %s] %s" % (ident, ex.message))
            #     rospy.logwarn("[EvaluatePoseGenerators.run() - %s] result: %s" % (ident, result))
            #     rospy.logwarn("[EvaluatePoseGenerators.run() - %s] hull: %s" % (ident, hull))
            #     rospy.logwarn("[EvaluatePoseGenerators.run() - %s] obstacles: %s" % (ident, obstacles))
            #     return

            # Evaluate who is best most of the time
            if hull_distance > d_hull:
                d_hull = hull_distance
                hull_ident = ident
            if obstacle_distance > d_obst:
                d_obst = obstacle_distance
                obst_ident = ident

        # Sometimes no obstacles are present, resulting in the same position for all generators,
        # we want to filter such cases
        obs_points, hull_points = self._generators[-1].extract_points(obs, flr.tables[0])
        valid_points = PoseGeneratorRosInterface.in_hull(Delaunay(hull_points[:, :2], qhull_options="Pp"),
                                                         obs_points[:, :2])
        purge_last_evaluation = len(valid_points) == 0
        if purge_last_evaluation:
            rospy.logdebug("[EvaluatePoseGenerators.run()] No obstacles in hull, we wont evaluate this sample")
            for g in self._generators:
                ident = g.get_name()
                self.dct_timing[ident].pop()
                self.dct_result[ident].pop()
                self.dct_lst_hull_distance[ident].pop()
                self.dct_lst_obstacle_distance[ident].pop()
            return

        self.dct_count_largest_hull_distance[hull_ident] += 1
        self.dct_count_largest_obstacle_distance[obst_ident] += 1

    def evaluate(self, print_it=False, ff=['.tex', '.pdf']):
        """
        Plot the gathered data
        :return: -
        :rtype:-
        """
        s_dir = rospy.get_param("~", "/home/grehl/Schreibtisch/PoseGeneratorImages")

        for k in self.dct_lst_hull_distance.keys():
            if len(self.dct_lst_hull_distance[k]) == 0:
                return

        rospy.loginfo("[EvaluatePoseGenerators.evaluate()] Number of largest distances to the hull:")
        for k in self.dct_count_largest_hull_distance:
            rospy.loginfo("%s\t%g" % (k, self.dct_count_largest_hull_distance[k]))
        rospy.loginfo("[EvaluatePoseGenerators.evaluate()] Number of largest distances to nearest obstacles:")
        for k in self.dct_count_largest_obstacle_distance:
            rospy.loginfo("%s\t%g" % (k, self.dct_count_largest_obstacle_distance[k]))

        n_bin = 25
        alpha = 0.75
        self.plot_hist(self.dct_lst_hull_distance, bins=n_bin, title=u'Abstand zur konvexen Hülle', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="hull_histogram")
        self.plot_hist(self.dct_lst_obstacle_distance, bins=n_bin, title=u'Abstand zum nächsten Hindernis', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="obstacle_histogram")
        self.plot_hist(self.dct_timing, bins=n_bin, title=u'Rechenzeit', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff, suffix="timing", save_dir=s_dir)

    def distance_to(self, lst_results, n_bin=25, alpha=0.75, print_it=False, show_it=False, ff=['.tex', '.pdf']):
        """
        Determine the distance to a list of results
        :param print_it: whether or not to print the plots
        :type print_it: bool
        :param show_it: whether or not to show_it the plots
        :type show_it: bool
        :param ff: file formats of the print
        :type ff: list
        :param alpha: Alpha channel of the bar plot
        :type alpha: float
        :param lst_results: list of results
        :type lst_results: list
        :param n_bin: number of bins used in bar plot histogram
        :type n_bin: int
        :return: -
        :rtype: -
        """
        dct_distances = {}
        for key in self.dct_result.keys():
            dct_distances[key] = []
        for key in dct_distances.keys():
            for i in range(0, len(lst_results)):
                d = np.linalg.norm(lst_results[i] - self.dct_result[key][i])
                if d != 0:
                    dct_distances[key].append(d)
        for key in self.dct_result.keys():
            if len(dct_distances[key]) == 0:
                del dct_distances[key]
        self.plot_hist(dct_distances, bins=n_bin, title=u'Abstand zur Ground Truth', alpha=alpha)
        if print_it:
            print_plt(file_formats=ff)
        if show_it:
            plt.show()

    def plot_heatmap(self, name='all', n_bin=20, print_it=False, show_it=False, ff=['.tex', '.pdf']):
        """
        Plot a heatmap of all results
        :param name:
        :type name: str
        :param n_bin: number of bins for the 2d histogram
        :type n_bin: int
        :param print_it: whether or not to print the plots
        :type print_it: bool
        :param show_it: whether or not to show_it the plots
        :type show_it: bool
        :param ff: file formats of the print
        :type ff: list
        :return: -
        :rtype: -
        """
        lst_names = []
        if 'all' in name:
            for g in self._generators:
                lst_names.append(g.get_name())
        else:
            lst_names.append(name)

        rospy.logdebug("[EvaluatePoseGenerators.plot_heatmap()] %s" % lst_names)
        for n in lst_names:
            values = np.asarray(self.dct_result[n])
            if len(values) == 0:
                break
            # See: https://docs.scipy.org/doc/numpy/reference/generated/numpy.histogram2d.html
            data, xedges, yedges = np.histogram2d(values[:, 0].T, values[:, 1].T, bins=n_bin, range=[[-1, 1], [-1, 1]])
            data = data.T
            a_title = "Heatmap " + n
            fig = plt.figure()
            ax = fig.add_subplot(111, title=a_title, aspect='equal', xlim=[-1, 1], ylim=[-1, 1])
            im = NonUniformImage(ax, interpolation='nearest')
            im.set_cmap(EvaluatePoseGenerators.C_MAP)
            im.set_clim(vmin=0, vmax=0.2*len(values))
            xcenters = (xedges[:-1] + xedges[1:]) / 2
            ycenters = (yedges[:-1] + yedges[1:]) / 2
            rospy.logdebug("[EvaluatePoseGenerators.plot_heatmap(%s)] data:\n %s" % (n, data))
            im.set_data(xcenters, ycenters, data)
            ax.images.append(im)
            plt.colorbar(im)
            if print_it:
                print_plt(file_formats=ff, suffix=a_title)
            if show_it:
                plt.show()

    @staticmethod
    def plot_hist(dct, **kwargs):
        """
        plot the given dictionary in a histogram
        :param dct: data
        :type dct: dict
        :return: -
        :rtype:-
        """
        my_colors = []
        for k in dct.keys():
            my_colors.append(PoseGeneratorRosView.get_color(k))
        plt.hist(dct.values(), color=my_colors, label=dct.keys())
        pd.DataFrame(data=dct).plot.hist(color=my_colors, **kwargs)
