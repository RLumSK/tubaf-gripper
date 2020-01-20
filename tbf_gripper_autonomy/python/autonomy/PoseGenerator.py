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
import sys
import abc
import six

from message_filters import Subscriber, Cache

from object_recognition_msgs.msg import TableArray, Table
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point

from pylab import *
import matplotlib.pyplot as plt



class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


@six.add_metaclass(abc.ABCMeta)
class PoseGeneratorRosInterface(object):
    """
    Abstract class to determine a valid pose on a floor plane for an object to set on.
    It manages the communication towards ROS.
    Any child class has to implement the specific method in '_generate()'.
    """

    def __init__(self, pub_topic):
        """
        Default constructor
        :param pub_topic: Topic where the calculated pose gets published
        :type pub_topic: str
        """
        _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
        _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")
        self.subsample = rospy.get_param("~sub_sample", 25)

        self._obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
        self._floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

        self.pub = rospy.Publisher(pub_topic, PoseStamped, queue_size=1)

        self._dbg_pub_projection = rospy.Publisher(_obstacle_topic+"_projected", MarkerArray, queue_size=1)

        self.result = [0, 0]
        self.obs_points = []
        self.hull_points = []


    def perform(self):
        """
        Main Loop - Reads messages and generates poses
        :return: -
        :rtype: -
        """
        while True:
            rospy.sleep(1.0)
            self.once()

    def once(self):
        """
        Generate pose and publish it
        :return: -
        :rtype: -
        """
        obstacles_msg = self._obstacle_cache.getLast()  # type: MarkerArray
        floor_msg = self._floor_cache.getLast()  # type: TableArray
        if obstacles_msg is None or floor_msg is None:
            return
        self.obs_points, self.hull_points = self.extract_points(obstacles_msg, floor_msg.tables[0])

        self.result = self._generate(self.obs_points + self.hull_points)  # type: list
        ps = PoseGeneratorRosInterface._as_pose_stamped(self.result, floor_msg.tables[0])
        self.pub.publish(ps)

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
            r = Rotation.from_quat([flr_msg.pose.orientation.x, flr_msg.pose.orientation.y, flr_msg.pose.orientation.z,
                                    flr_msg.pose.orientation.w])
            result = np.matmul(r.as_dcm(), [x, y, z])  # type: np.ndarray
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
        # rospy.logdebug("[PoseGeneratorRosInterface.calculate_plane_equation()] Plane equation: %gx + %gy + %gz + %g = 0" %
        #                (a, b, c, d))
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
        # TODO: filter near on plane?
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
    def _generate(self, lst_points):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_points: list with all points projected on the floor plane
        :type lst_points: list
        :return: tuple of the position [x, y, z]
        :rtype: list
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
            for point in obstacle.points:  # type: Point
                i += 1
                if i % self.subsample != 0:
                    continue
                prjt_pnt = PoseGeneratorRosInterface.point_on_plane(point, plane_parameters)
                obs_points.append(prjt_pnt)
                point.x = prjt_pnt[0]
                point.y = prjt_pnt[1]
                point.z = prjt_pnt[2]
                dbg_pnts.append(point)
            obstacle.points = []

        # Add convex hull to dataset
        hull_pnts = []
        for i in range(0, len(flr_msg.convex_hull)):
            prjt_pnt = PoseGeneratorRosInterface.point_on_plane(flr_msg.convex_hull[i], plane_parameters)
            hull_pnts.append(prjt_pnt)
            a_point = Point()
            a_point.x = prjt_pnt[0]
            a_point.y = prjt_pnt[1]
            a_point.z = prjt_pnt[2]
            dbg_pnts.append(a_point)

        dbg_markers.append(obs_msg.markers[0])
        obs_msg.markers[0].points = dbg_pnts
        self._dbg_pub_projection.publish(obs_msg)
        return [obs_points, hull_pnts]

    @staticmethod
    def _as_pose_stamped(lst_pos, floor_msg):
        """
        Generate the target pose for the given position by using the orientation of the plane
        :param lst_pos: position [x, y, z]
        :type lst_pos: list
        :param floor_msg: floor plane
        :type floor_msg: Table
        :return: target pose
        :rtype: PoseStamped
        """
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


class PcaPoseGenerator(PoseGeneratorRosInterface):

    def __init__(self, topic):
        """
        Default constructor
        :param topic: Topic where the calculated pose gets published
        :type topic: str
        """
        super(PcaPoseGenerator, self).__init__(topic)

    def _generate(self, lst_points):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_points: list with all points projected on the floor plane
        :type lst_points: list
        :return: tuple of the position [x, y, z]
        :rtype: list
        """
        rospy.logdebug("[PcaPoseGenerator._generate()] Starting")
        all_points = lst_points

        # see: https://scikit-learn.org/stable/auto_examples/decomposition/plot_pca_iris.html
        from sklearn import decomposition
        pca = decomposition.PCA(n_components=2)
        pca.fit(all_points)
        pca_values = pca.transform(all_points)
        pca_x = pca_values[:, 0]
        pca_y = pca_values[:, 1]

        def get_largest_gap(series):
            series.sort()
            max_dist = 0.0
            gap = [0.0, 0.0]
            for i in range(1, len(series)):
                dist = np.abs(series[i] - series[i - 1])
                if dist > max_dist:
                    max_dist = dist
                    gap = [series[i - 1], series[i]]
                    rospy.logdebug("[PcaPoseGenerator._generate().get_largest_gap()] max_dist: %g" % max_dist)
            # rospy.logdebug("[PcaPoseGenerator._generate().get_largest_gap()] max_dist: %_g\t gap: %s" % (max_dist, gap))
            return gap

        def get_sample_in_gap(_g):
            return _g[0] + 0.5 * (_g[1] - _g[0])

        g = [get_sample_in_gap(get_largest_gap(pca_x)), get_sample_in_gap(get_largest_gap(pca_y))]
        # rospy.logdebug("[PcaPoseGenerator._generate()] g: %s" % g)
        p = pca.inverse_transform(g)
        # rospy.logdebug("[PcaPoseGenerator._generate()] p: %s" % p)

        return p


class DelaunayPoseGenerator(PoseGeneratorRosInterface):

    def __init__(self, topic):
        """
        Default constructor
        :param topic: Topic where the calculated pose gets published
        :type topic: str
        """
        super(DelaunayPoseGenerator, self).__init__(topic)

    def _generate(self, lst_points):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_points: list with all points projected on the floor plane
        :type lst_points: list
        :return: tuple of the position [x, y, z]
        :rtype: list
        """
        rospy.logdebug("[DelaunayPoseGenerator._generate()] Starting")
        all_points = lst_points
        points = np.asarray(all_points)

        from scipy.spatial import Delaunay
        dln = Delaunay(points[:, :2], incremental=False, qhull_options="Pp")  # type: Delaunay

        def distance(p1, p2):
            d = p2 - p1
            return (d[0]**2+d[1]**2)**0.5

        def area(pnts):
            assert len(pnts) == 3
            i_a = distance(pnts[0], pnts[1])
            i_b = distance(pnts[0], pnts[2])
            i_c = distance(pnts[1], pnts[2])
            # formula of Heron: https://en.wikipedia.org/wiki/Heron%27s_formula#Numerical_stability
            a = max(i_a, i_b, i_c)
            b = np.median([i_a, i_b, i_c])
            c = min(i_a, i_b, i_c)
            return 0.25 * ((a+(b+c))*(c-(a-b))*(c+(a-b))*(a+(b-c)))**0.5

        max_distance = 0
        max_idx = [0, 0, 0]
        for triangle in dln.simplices:  # type: list
            p0 = dln.points[triangle[0]]
            p1 = dln.points[triangle[1]]
            p2 = dln.points[triangle[2]]

            v = area([p0, p1, p2])
            if max_distance < v:
                max_distance = v
                max_idx = [triangle[0], triangle[1], triangle[2]]
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

        g = get_center_point([dln.points[max_idx[0]],  dln.points[max_idx[1]],  dln.points[max_idx[2]]])

        return g


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
        self.n_bins = rospy.get_param("~n_bins", 100)

    def _generate(self, lst_points):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_points: list with all points projected on the floor plane
        :type lst_points: list
        :return: tuple of the position [x, y, z]
        :rtype: list
        """
        # see: https://stackoverflow.com/questions/41577705/how-does-2d-kernel-density-estimation-in-python-sklearn-work
        from sklearn.neighbors import KernelDensity

        def kde2D(X,  bandwidth=0.25, xbins=100j, ybins=100j, **kwargs):
            """Build 2D kernel density estimate (KDE)."""
            x = X[:, 0]
            y = X[:, 1]
            # create grid of sample locations (default: 100x100)
            xx, yy = np.mgrid[x.min():x.max():xbins, y.min():y.max():ybins]
            xy_sample = np.vstack([yy.ravel(), xx.ravel()]).T
            xy_train = np.vstack([y, x]).T

            kde_skl = KernelDensity(kernel='gaussian', bandwidth=bandwidth, **kwargs)
            kde_skl.fit(xy_train)

            # score_samples() returns the log-likelihood of the samples
            z = np.exp(kde_skl.score_samples(xy_sample))
            z_i = np.argmin(z)
            z_x = xy_sample[z_i, 1]
            z_y = xy_sample[z_i, 0]
            rospy.logdebug("[DistanceFieldPoseGenerator._generate().kde2D] z_i=%g, value=%g" % (z_i, z[z_i]))
            rospy.logdebug("[DistanceFieldPoseGenerator._generate().kde2D] z= %g, %g" % (z_x, z_y))

            return [z_x, z_y]

        return kde2D(np.asarray(lst_points), xbins=self.n_bins*1j, ybins=self.n_bins*1j)
        #
        # from sklearn.preprocessing import KBinsDiscretizer
        # from scipy.sparse.csr import csr_matrix
        # enc = KBinsDiscretizer(n_bins=self.n_bins, encode='ordinal', strategy='uniform')  # type: KBinsDiscretizer
        # pnts_binned = enc.fit_transform(np.asarray(lst_points)[:, :2])  # type: np.ndarray
        # rospy.logdebug("[DistanceFieldPoseGenerator._generate()] #NNZ %g/%g)" % (len(pnts_binned), enc.n_bins))
        # from scipy.ndimage import distance_transform_edt
        # dst_field = distance_transform_edt(pnts_binned)
        # print dst_field
        # index = np.argmax(dst_field)
        # rospy.logdebug("[DistanceFieldPoseGenerator._generate()] d= %s at (%s)" % (np.max(dst_field), index))


class ViewPoseGenerator(object):

    def __init__(self, lst_generator):
        """
        Default constructor for PoseGeneratorRosInterface
        :param generator: object to view
        :type generator: list of PoseGeneratorRosInterface
        """

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.axes = {}
        self.lst_generator = lst_generator
        for generator in self.lst_generator:  # type: PoseGeneratorRosInterface
            while len(generator.obs_points) == 0:
                rospy.sleep(1.0)
                generator.once()
                self.axes[id(generator)], = plot(generator.result[0], generator.result[1], 'o',
                                                label=generator.__class__.__name__)
        as_array = np.asarray(lst_generator[-1].obs_points)
        self.obs_axis, = plot(as_array[:, 0], as_array[:, 1], '.')
        as_array = np.asarray(lst_generator[-1].hull_points)
        self.hull_axis, = plot(as_array[:, 0], as_array[:, 1], '+')
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    def view(self):
        """
        view it
        """
        for generator in self.lst_generator:  # type: PoseGeneratorRosInterface
            self.axes[id(generator)].set_xdata(generator.result[0])
            self.axes[id(generator)].set_ydata(generator.result[1])
        as_array = np.asarray(self.lst_generator[-1].obs_points)
        self.obs_axis.set_xdata(as_array[:, 0])
        self.obs_axis.set_ydata(as_array[:, 1])
        as_array = np.asarray(self.lst_generator[-1].hull_points)
        self.hull_axis.set_xdata(as_array[:, 0])
        self.hull_axis.set_ydata(as_array[:, 1])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        rospy.logdebug("[ViewPoseGenerator.view()] Finished")


if __name__ == '__main__':
    rospy.init_node("test_SetPoseGenerator", log_level=rospy.DEBUG)
    pub_topic = rospy.get_param("~pub_topic", "target_pose")
    pca = PcaPoseGenerator(pub_topic+"_pca")
    dln = DelaunayPoseGenerator(pub_topic+"_delaunay")
    kde = MinimalDensityEstimatePoseGenerator(pub_topic+"_kde")

    # import threading
    # pca_thread = threading.Thread(target=pca.perform)
    # dln_thread = threading.Thread(target=dln.perform)
    # pca_thread.daemon = True
    # dln_thread.daemon = True
    # pca_thread.start()
    # dln_thread.start()
    plt.ion()
    lst = [kde, pca, dln]
    view = ViewPoseGenerator(lst)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        for g in lst:
            g.once()
        view.view()
        r.sleep()
