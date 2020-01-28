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
import signal
import abc
from six import add_metaclass

from message_filters import Subscriber, Cache

from object_recognition_msgs.msg import TableArray, Table
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point

from pylab import *
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial import Delaunay


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


@add_metaclass(abc.ABCMeta)
class PoseGeneratorRosInterface:
    """
    Abstract class to determine a valid pose on a floor plane for an object to set on.
    It manages the communication towards ROS.
    Any child class has to implement the specific method in '_generate()'.
    """

    HULL_THRESHOLD = 6

    def __init__(self, pub_topic):
        """
        Default constructor
        :param pub_topic: Topic where the calculated pose gets published
        :type pub_topic: str
        """
        _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
        _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")
        self.subsample = rospy.get_param("~sub_sample", 0.2)

        self._obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
        self._floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

        self.pub = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)

        self._dbg_pub_projection = rospy.Publisher(_obstacle_topic + "_projected", MarkerArray, queue_size=1)

        self.result = np.asarray([0, 0, 0])  # type: np.ndarray
        self.obs_points = []
        self.hull_points = []
        self.lines = []

    def perform(self):
        """
        Main Loop - Reads messages and generates poses
        :return: -
        :rtype: -
        """
        while True:
            rospy.sleep(0.2)
            self.once()

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
        :return: -
        :rtype: -
        """
        t = current_time
        if t is None:
            t = self._obstacle_cache.getLatestTime()
        if obstacles_msg is None:
            obstacles_msg = self._obstacle_cache.getElemBeforeTime(t)  # type: MarkerArray
        if floor_msg is None:
            floor_msg = self._floor_cache.getElemBeforeTime(t)  # type: TableArray
        if not self.check_messages(obstacles_msg, floor_msg):
            # rospy.loginfo("[PoseGeneratorRosInterface.once()] messages not suitable\n%s\n---\n%s" %
            #               (obstacles_msg, floor_msg))
            return

        self.obs_points, self.hull_points = self.extract_points(obstacles_msg, floor_msg.tables[0])

        valid_points = PoseGeneratorRosInterface.in_hull(Delaunay(self.hull_points[:, :2], qhull_options="Pp"),
                                                         self.obs_points[:, :2])  # type: np.ndarray
        if len(valid_points) == 0:
            rospy.logwarn("[PoseGeneratorRosInterface.once()] No obstacle points inside the hull")
            self.publish_default(floor_msg)
            rospy.sleep(1.0)  # next iteration most likely also without obstacles
            return

        self.result = self._generate(valid_points, hull=self.hull_points[:, :2])  # type: np.ndarray
        ps = PoseGeneratorRosInterface._as_pose_stamped(self.result.tolist(), floor_msg.tables[0])
        if len(self.result) == 0:
            self.result = np.hstack([self.result, ps.pose.position.x])
        if len(self.result) == 1:
            self.result = np.hstack([self.result, ps.pose.position.y])
        if len(self.result) == 2:
            self.result = np.hstack([self.result, ps.pose.position.z])
        self.pub.publish(ps)

    def publish_default(self, floor_message):
        """
        Publish the center of the floor as pose
        :param floor_message: floor
        :type floor_message: TableArray
        :return: pose
        :rtype: PoseStamped
        """
        ps = PoseStamped()
        ps.header = floor_message.header
        ps.pose = floor_message.tables[0].pose
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
            rospy.logwarn("[PoseGeneratorRosInterface.check_messages()] One or both messages is/are None")
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
                if len(marker.points) != 0:
                    return True
        rospy.logwarn("[PoseGeneratorRosInterface.check_messages()] No Obstacles given")
        self.publish_default(flr_msg)
        return False

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
        :return: list with all points projected on the floor plane
        :rtype: list
        """
        obs_points = []
        dbg_pnts = []
        dbg_markers = []
        plane_parameters = PoseGeneratorRosInterface.calculate_plane_equation(flr_msg)
        for obstacle in obs_msg.markers:  # type: Marker
            i = 0
            if self.subsample > 1.0:
                ss = self.subsample
            else:
                ss = int(len(obstacle.points) * self.subsample)
            for point in obstacle.points:  # type: Point
                i += 1
                if i % ss != 0 and len(obs_points) != 0:
                    continue
                prjt_pnt = PoseGeneratorRosInterface.point_on_plane(point, plane_parameters)
                obs_points.append(prjt_pnt)
                point.x = prjt_pnt[0]
                point.y = prjt_pnt[1]
                point.z = prjt_pnt[2]
                dbg_pnts.append(point)

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
            self._dbg_pub_projection.publish(obs_msg)

        hull_pnts = np.asarray(hull_pnts)
        obs_points = np.asarray(obs_points)
        return [obs_points, hull_pnts]

    @staticmethod
    def _as_pose_stamped(lst_pos, floor_msg):
        # type: (list, Table) -> PoseStamped
        """
        Generate the target pose for the given position by using the orientation of the plane
        :param lst_pos: position [x, y, z]
        :type lst_pos: list
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


class PcaPoseGenerator(PoseGeneratorRosInterface):

    def __init__(self, topic):
        """
        Default constructor
        :param topic: Topic where the calculated pose gets published
        :type topic: str
        """
        super(PcaPoseGenerator, self).__init__(topic)

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
        if lst_obs_points is None or len(lst_obs_points) == 0:
            rospy.logwarn("[PcaPoseGenerator._generate()] No obstacle points given")
            return np.asarray([0.0, 0.0])

        if hull is None:
            hull = []
        rospy.logdebug("[PcaPoseGenerator._generate()] Starting")

        if len(lst_obs_points) < 3:
            rospy.logwarn("[PcaPoseGenerator._generate()] Not enough obstacle points: %g" % len(lst_obs_points))
            return np.asarray([])

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

        limits = [get_largest_gap(pca_x, hull_values), get_largest_gap(pca_y, hull_values)]
        g_sample = [get_sample_in_gap(limits[0]), get_sample_in_gap(limits[0])]
        h = Delaunay(np.asarray(hull_values), qhull_options="Pp")
        while len(PoseGeneratorRosInterface.in_hull(h, [g_sample])) == 0:
            new_limits = [get_largest_gap(pca_x, blacklst=limits[0]), get_largest_gap(pca_y, blacklst=limits[1])]
            limits[0].extend(new_limits[0])
            limits[1].extend(new_limits[1])
            g_sample = [get_sample_in_gap(new_limits[0]), get_sample_in_gap(new_limits[0])]

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


class DelaunayPoseGenerator(PoseGeneratorRosInterface):

    def __init__(self, topic):
        """
        Default constructor
        :param topic: Topic where the calculated pose gets published
        :type topic: str
        """
        super(DelaunayPoseGenerator, self).__init__(topic)

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


class MinimalDensityEstimatePoseGenerator(PoseGeneratorRosInterface):
    """
    Determine the Pose using KDE and discretization
    """

    def __init__(self, topic):
        """
        Default constructor
        :param topic: Topic where the calculated pose gets published
        :type topic: str
        """
        super(MinimalDensityEstimatePoseGenerator, self).__init__(topic)
        self.n_bins = rospy.get_param("~n_bins", 10)

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

        from sklearn.neighbors import KernelDensity

        def kde2D(X, area=np.asarray([]), bandwidth=0.25, xbins=100j, ybins=100j, **kwargs):
            """Build 2D kernel density estimate (KDE)."""
            x_pnts = X[:, 0]
            y_pnts = X[:, 1]
            # create grid of valid_sample locations (default: 100x100)
            if len(area) == 0:
                x = x_pnts
                y = y_pnts
            else:
                x = area[:, 0]
                y = area[:, 1]
            xx, yy = np.mgrid[x.min():x.max():xbins, y.min():y.max():ybins]
            xy_sample = np.vstack([xx.ravel(), yy.ravel()]).T

            if len(area) > 0:
                valid_sample = PoseGeneratorRosInterface.in_hull(Delaunay(area[:, :2], qhull_options="Pp"), xy_sample)
            else:
                valid_sample = xy_sample
            rospy.logdebug("[DistanceFieldPoseGenerator._generate().kde2D] len(area) = %g, len(valid_sample) = %g/%g" %
                           (len(area), len(valid_sample), (xbins * ybins).real * -1))

            xy_train = np.vstack([x_pnts, y_pnts]).T

            kde_skl = KernelDensity(kernel='gaussian', bandwidth=bandwidth, **kwargs)
            kde_skl.fit(xy_train)

            # score_samples() returns the log-likelihood of the samples
            z = np.exp(kde_skl.score_samples(valid_sample))
            z_i = np.argmin(z)
            z_x = valid_sample[z_i, 0]
            z_y = valid_sample[z_i, 1]
            rospy.logdebug("[DistanceFieldPoseGenerator._generate().kde2D] z_i=%s, value=%g" % (z_i, z[z_i]))
            rospy.logdebug("[DistanceFieldPoseGenerator._generate().kde2D] z= %g, %g" % (z_x, z_y))

            return np.asarray([z_x, z_y])

        [x, y] = kde2D(lst_obs_points, area=hull, xbins=self.n_bins * 1j, ybins=self.n_bins * 1j)
        return np.asarray([x, y])


class ViewPoseGenerators(object):

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
        if "stacle" in name:
            return 'C0'
        elif "roundarea" in name:
            return 'C1'
        elif "elaunay" in name:
            return 'C2'
        elif "inimalDensityEstimate" in name:
            return 'C3'
        elif "ca" in name:
            return 'C4'
        else:
            return 'C5'

    def __init__(self, lst_generator):
        """
        Default constructor for PoseGeneratorRosInterface
        :param lst_generator: list with objects to view
        :type lst_generator: list of PoseGeneratorRosInterface
        """

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.pnt_axes = {}
        self.lne_axis = {}
        self.lst_generator = lst_generator
        for generator in self.lst_generator:  # type: PoseGeneratorRosInterface
            while len(generator.obs_points) == 0:
                rospy.sleep(1.0)
                generator.once()
                name = generator.__class__.__name__
                self.pnt_axes[id(generator)], = self.ax.plot(generator.result[0], generator.result[1], 'o', ms=12,
                                                             zorder=4.0, color=ViewPoseGenerators.get_color(name),
                                                             label=name)
                color = self.pnt_axes[id(generator)].get_color()
                for lne in generator.lines:
                    if not id(generator) in self.lne_axis:
                        self.lne_axis[id(generator)] = []
                    self.lne_axis[id(generator)].append(
                        self.ax.plot([lne[0][0], lne[1][0]], [lne[0][1], lne[1][1]], ':', color=color, alpha=0.75,
                                     zorder=5.0))
        as_array = np.asarray(lst_generator[-1].obs_points)
        name = "Obstacles"
        self.obs_axis, = self.ax.plot(as_array[:, 0], as_array[:, 1], '.', label=name, zorder=3.5,
                                      color=ViewPoseGenerators.get_color(name))
        hull = lst_generator[-1].hull_points.tolist()
        hull.append(hull[0])
        as_array = np.asarray(hull)
        name = "Groundarea"
        self.hull_axis, = self.ax.plot(as_array[:, 0], as_array[:, 1], '+--', label=name, zorder=4.5,
                                       color=ViewPoseGenerators.get_color(name))

        # Text
        if lst_generator[-1].subsample > 1.0:
            self.fig.text(0.15, 0.01, "Subsample every " + str(lst_generator[-1].subsample) + "th")
        else:
            self.fig.text(0.15, 0.01, "Subsample " + str(lst_generator[-1].subsample * 100.0) + "%")

        # Place a legend above this subplot, expanding itself to
        # fully use the given bounding box.
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
                   ncol=2, mode="expand", borderaxespad=0.)
        plt.ion()
        plt.show()

    def view(self):
        """
        view it
        """
        try:
            # Update Obstacles
            as_array = np.asarray(self.lst_generator[-1].obs_points)
            if len(as_array) != 0 and len(as_array[0]) > 0:
                self.obs_axis.set_xdata(as_array[:, 0])
                self.obs_axis.set_ydata(as_array[:, 1])
            else:
                return
            # Update Hull
            hull = self.lst_generator[-1].hull_points.tolist()
            hull.append(hull[0])
            as_array = np.asarray(hull)
            if len(as_array) != 0 and len(as_array[0]) > 0:
                self.hull_axis.set_xdata(as_array[:, 0])
                self.hull_axis.set_ydata(as_array[:, 1])
            else:
                self.hull_axis.set_xdata(np.asarray([]))
                self.hull_axis.set_ydata(np.asarray([]))
            # Update generated positions
            for generator in self.lst_generator:  # type: PoseGeneratorRosInterface
                self.pnt_axes[id(generator)].set_xdata(generator.result[0])
                self.pnt_axes[id(generator)].set_ydata(generator.result[1])

                if not id(generator) in self.lne_axis:
                    continue

                for lne2d, lne in zip(self.lne_axis[id(generator)], generator.lines):  # type: (plt.Line2D, list)
                    lne2d[0].set_data([lne[0][0], lne[1][0]], [lne[0][1], lne[1][1]])
                # # Remove previous lines:
                # for lne2d in self.lne_axis[id(generator)]:  # type: plt.Line2D
                #     self.ax.lines.remove(lne2d[0])
                # self.lne_axis[id(generator)] = []
                # # Add new lines
                # for lne in generator.lines:
                #     self.lne_axis[id(generator)].append(
                #         self.ax.plot([lne[0][0], lne[1][0]], [lne[0][1], lne[1][1]], 'g-'))

            # Redraw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            # See: https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib
            plt.pause(0.1)
        except Exception as ex:
            rospy.logwarn("[ViewPoseGenerators.view()] Exception occurred\n%s" % ex)
        rospy.logdebug("[ViewPoseGenerators.view()] Finished")


class EvaluatePoseGenerators(object):
    """
    Evaluation for a PoseGeneratorRosInterface instance
    """

    @staticmethod
    def calc_min_distance(point, lst_points):
        """
        Given a point, calculate the minimal distance to a list of points
        :param point: point of interest
        :type point: np.ndarray
        :param lst_points: list of points to test against
        :type lst_points: list
        :return: minimal distance
        :rtype: float
        """
        # See: https://scikit-learn.org/stable/modules/generated/sklearn.metrics.pairwise_distances_argmin_min.html
        from sklearn.metrics import pairwise_distances_argmin_min
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] point = %s" % point)
        rospy.logdebug("[EvaluatePoseGenerators.calc_min_distance()] len(lst_points) = %s" % len(lst_points))
        min_i, distances = pairwise_distances_argmin_min(point, lst_points)
        # rospy.loginfo("[EvaluatePoseGenerators.calc_min_distance()] min_i = %g" % min_i)
        # rospy.loginfo("[EvaluatePoseGenerators.calc_min_distance()] distances = %s" % distances)
        return distances[0]

    def __init__(self, generators):
        """
        Default constructor
        :param generators: pose generators
        :type generators: list of PoseGeneratorRosInterface
        """
        self._generators = generators

        self.dct_lst_min_hull_distance = {}
        self.dct_lst_min_obstacle_distance = {}
        self.dct_count_largest_hull_distance = {}
        self.dct_count_largest_obstacle_distance = {}

        for g in self._generators:  # type: PoseGeneratorRosInterface
            ident = EvaluatePoseGenerators.get_ident(g)
            self.dct_lst_min_hull_distance[ident] = []
            self.dct_lst_min_obstacle_distance[ident] = []
            self.dct_count_largest_hull_distance[ident] = 0
            self.dct_count_largest_obstacle_distance[ident] = 0

        self.view = ViewPoseGenerators(self._generators)

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
            self.view.view()
            rospy.sleep(timeout)
            i += 1
        self.evaluate()

    @staticmethod
    def get_ident(obj):
        return str(type(obj)).split(".")[-1][:-2]

    def run(self):
        """
        Trigger given generators and save evaluation parameters
        :return: -
        :rtype: -
        """
        while True:
            obs, flr = self._generators[0].get_messages()
            if self._generators[0].check_messages(obs, flr):
                break
            rospy.sleep(1.0)

        d_hull = 0
        d_obst = d_hull
        hull_ident = ""
        obst_ident = hull_ident

        for g in self._generators:  # type: PoseGeneratorRosInterface
            ident = EvaluatePoseGenerators.get_ident(g)
            g.once(obstacles_msg=obs, floor_msg=flr)
            result = np.asarray([g.result])
            obstacles = g.obs_points
            hull = g.hull_points
            if len(hull) == 0 or len(obstacles) == 0:
                return
            try:
                hull_distance = EvaluatePoseGenerators.calc_min_distance(result, hull)
                obstacle_distance = EvaluatePoseGenerators.calc_min_distance(result, obstacles)
                self.dct_lst_min_hull_distance[ident].append(hull_distance)
                self.dct_lst_min_obstacle_distance[ident].append(obstacle_distance)
                rospy.logdebug("[EvaluatePoseGenerators.run() - %s] Min(Distance2Hull) = %g" %
                               (ident, hull_distance))
                rospy.logdebug("[EvaluatePoseGenerators.run() - %s] Min(Distance2Obstacles) = %g" %
                               (ident, obstacle_distance))
            except ValueError as ex:
                rospy.logwarn("[EvaluatePoseGenerators.run() - %s] %s" % (ident, ex.message))
                rospy.logwarn("[EvaluatePoseGenerators.run() - %s] result: %s" % (ident, result))
                rospy.logwarn("[EvaluatePoseGenerators.run() - %s] hull: %s" % (ident, hull))
                rospy.logwarn("[EvaluatePoseGenerators.run() - %s] obstacles: %s" % (ident, obstacles))
                return

            # Evaluate who is best most of the time
            if hull_distance > d_hull:
                d_hull = hull_distance
                hull_ident = ident
            if obstacle_distance > d_obst:
                d_obst = obstacle_distance
                obst_ident = ident

        self.dct_count_largest_hull_distance[hull_ident] += 1
        self.dct_count_largest_obstacle_distance[obst_ident] += 1

    def evaluate(self):
        """
        Plot the gathered data
        :return: -
        :rtype:-
        """
        for k in self.dct_lst_min_hull_distance.keys():
            if len(self.dct_lst_min_hull_distance[k]) == 0:
                return
        df_hull = pd.DataFrame(data=self.dct_lst_min_hull_distance)
        df_obst = pd.DataFrame(data=self.dct_lst_min_obstacle_distance)

        if df_hull.empty or df_obst.empty:
            return

        rospy.loginfo("[EvaluatePoseGenerators.evaluate()] Number of largest distances to the hull:")
        for k in self.dct_count_largest_hull_distance:
            rospy.loginfo("%s\t%g" % (k, self.dct_count_largest_hull_distance[k]))
        rospy.loginfo("[EvaluatePoseGenerators.evaluate()] Number of largest distances to nearest obstacles:")
        for k in self.dct_count_largest_obstacle_distance:
            rospy.loginfo("%s\t%g" % (k, self.dct_count_largest_obstacle_distance[k]))

        my_colors = []
        for k in self.dct_lst_min_hull_distance.keys():
            my_colors.append(ViewPoseGenerators.get_color(k))
        n_bin = 20
        df_hull.plot.hist(bins=n_bin, rot=0, title='Distance to hull', color=my_colors)
        df_obst.plot.hist(bins=n_bin, rot=0, title='Distance to nearest obstacle', color=my_colors)

        plt.draw()
        plt.pause(0.01)


if __name__ == '__main__':
    rospy.init_node("test_SetPoseGenerator", log_level=rospy.INFO)
    rospy.logdebug("{test_SetPoseGenerator.main()} starting")
    pub_topic = rospy.get_param("~pub_topic", "target_pose")

    pca = PcaPoseGenerator(pub_topic + "_pca")
    dln = DelaunayPoseGenerator(pub_topic + "_dln")
    kde = MinimalDensityEstimatePoseGenerator(pub_topic + "_kde")

    evaluation = EvaluatePoseGenerators([pca, dln, kde])
    evaluation.perform(samples=rospy.get_param("~n_samples", 1000))

    # import threading
    # pca_thread = threading.Thread(target=pca.perform)
    # dln_thread = threading.Thread(target=dln.perform)
    # kde_thread = threading.Thread(target=kde.perform)
    # pca_thread.daemon = True
    # dln_thread.daemon = True
    # kde_thread.daemon = True
    # pca_thread.start()
    # dln_thread.start()
    # kde_thread.start()

    rospy.spin()

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
