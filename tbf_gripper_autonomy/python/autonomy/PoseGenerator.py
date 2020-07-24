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
import sys

if sys.version_info.major < 3:
    from __builtin__ import staticmethod

import rospy
import numpy as np
import quaternion
import signal
import abc
import datetime

from six import add_metaclass
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors

from object_recognition_msgs.msg import TableArray, Table
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Point, Pose

# from pylab import *
#
#
# import matplotlib

# matplotlib.use('TKAgg')
from pylab import *
import matplotlib.pyplot as plt
from matplotlib import ticker

if sys.version_info.major < 3:
    # see: https://stackoverflow.com/questions/55554352/import-of-matplotlib2tikz-results-in-syntaxerror-invalid-syntax
    try:
        import matplotlib2tikz as mpl2tkz
    except SyntaxError:
        # also: https://stackoverflow.com/questions/55554352/import-of-matplotlib2tikz-results-in-syntaxerror-invalid-syntax
        pass
else:
    import tikzplotlib as mpl2tkz


def pose_to_array(pose_msg, dst=None):
    """
    Convert a pose message to a 4x4 numpy array representing the affine transform from the pose
    :param pose_msg: the geometry_msgs/Pose message object
    :param dst: optional 4x4 output numpy array
    :return: dst or a newly created 4x4 numpy array
    :rtype: np.array
    """
    q = pose_msg.orientation
    q = np.quaternion(q.w, q.x, q.y, q.z)
    if dst is None:
        dst = np.eye(4)
    dst[:3, :3] = quaternion.as_rotation_matrix(q)
    dst[0, 3] = pose_msg.position.x
    dst[1, 3] = pose_msg.position.y
    dst[2, 3] = pose_msg.position.z
    return dst


def array_to_pose(arr):
    """
    Convert 4x4 numpy array representing the affine transform to a a pose message
    :param arr: 4x4 numpy array
    :return: the geometry_msgs/Pose message object
    :rtype: Pose
    """
    q = quaternion.from_rotation_matrix(arr[:3, :3])
    pose_msg = Pose()
    pose_msg.position.x = arr[0, 3]
    pose_msg.position.y = arr[1, 3]
    pose_msg.position.z = arr[2, 3]
    pose_msg.orientation.x = q.x
    pose_msg.orientation.y = q.y
    pose_msg.orientation.z = q.z
    pose_msg.orientation.w = q.w
    return pose_msg


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

DF_OBS_TOPIC = "/ork/tabletop/clusters"
DF_FLR_TOPIC = "/ork/floor_plane"
DF_PUB_TOPIC = "/generator_output_pose"
DF_SUB_SAMPLE = 0.01
DF_ENABLE_ROS = True
DF_N_BINS = 10
DF_MC_RASTER = 32
DF_MC_WO = 0.7
DF_PLT_SAVE_DIR = "/out/plots"


@add_metaclass(abc.ABCMeta)
class PoseGeneratorRosInterface:
    """
    Abstract class to determine a valid pose on a floor plane for an object to set on.
    It manages the communication towards ROS.
    Any child class has to implement the specific method in '_generate()'.
    """

    HULL_THRESHOLD = 6

    @staticmethod
    def transform_points_to_plane(pnts, floor_msg):
        """
        Transform a given set of points to a plane using a affine transformation
        :param pnts: list of n three-dimensional points (nx3)
        :type pnts: list of Point
        :param floor_msg: plane as table
        :type floor_msg: Table
        :return: list with points (nx3)
        :rtype: np.ndarray
        """
        A = pose_to_array(floor_msg.pose)
        t = np.ones((4, len(pnts)))
        t[:3, :] = np.asarray(pnts).T
        x = np.matmul(np.linalg.inv(A), t)
        x[2, :] = np.zeros(len(pnts))
        return np.matmul(A, x).T[:, :3]

    @staticmethod
    def transform_point_from_plane(pnt, floor_msg):
        """
        Transform a given point from a plane to the camera coordinate frame using a affine transformation
        :param pnts: three-dimensional point
        :type pnts: np.ndarray
        :param floor_msg: plane as table
        :type floor_msg: Table
        :return: affine transformation as 4x4 matrix
        :rtype: np.ndarray
        """
        if not isinstance(pnt, np.ndarray):
            pnt = np.asarray(pnt)
        rospy.loginfo("[PoseGeneratorRosInterface.transform_point_from_plane()] pnt: %s" % pnt)
        t = np.hstack([pnt, 1])

        M = pose_to_array(floor_msg.pose)  # from camera to plane, z is up (normal to plane)
        # We need the inverse of this,
        # see: http://negativeprobability.blogspot.com/2011/11/affine-transformations-and-their.html
        # and: https://math.stackexchange.com/questions/152462/inverse-of-transformation-matrix
        Pi = np.linalg.inv(M[:3, :3])
        v = pnt + np.matmul(M[:3, 3], np.asarray([1, 1, 0]))  # ???M[:3, 3] +
        # T[2] = 0 # set z=0
        Mi = np.eye(4)
        Mi[:3, :3] = Pi
        Mi[:3, 3] = np.matmul(-Pi, v.T)
        u = np.matmul(Mi, np.hstack([v, 1]).T).T
        rospy.loginfo("[PoseGeneratorRosInterface.transform_point_from_plane()] returning %s" % u)

        # Ai[:3, 3] = Ai[:3, 3] + x[:3]
        return u

    @staticmethod
    def transform_hull_to_camera(hull, floor_msg):
        """
        Transform a given set of points to a plane using a affine transformation
        :param pnts: list of n three-dimensional points (nx3)
        :type pnts: list of Point
        :param floor_msg: plane as table
        :type floor_msg: Table
        :return: list with points (nx3)
        :rtype: np.ndarray
        """
        A = pose_to_array(floor_msg.pose)
        t = np.ones((4, len(hull)))
        t[:3, :] = np.asarray(hull).T
        return np.matmul(A, t).T[:, :3]

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
        rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] point = %s" % point)
        rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] len(lst_points) = %s" % len(lst_points))
        rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] lst_points = %s" % lst_points)
        rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] mode = %s" % mode)
        if "PP" in mode:
            nn = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(
                lst_points)  # type: NearestNeighbors
            tmp = nn.kneighbors([point])
            distance, i_min = tmp[0][0][0], tmp[1][0][0]
            rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] distance: %s" % distance)
            rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] i_min: %s" % i_min)
            rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance()] nn: %s" % lst_points[i_min])
            return distance, lst_points[i_min]
        elif "PL" in mode:
            min_d = float('inf')
            min_f = np.asarray([0, 0])
            p1 = point[0:2]
            p3 = np.asarray(lst_points[-1])[0:2]
            for i in range(0, len(lst_points)):
                p2 = p3
                p3 = np.asarray(lst_points[i])[0:2]
                f = PoseGeneratorRosInterface.calc_perpendicular_point(p1, np.asarray([p2, p3]))
                d = np.linalg.norm(f - p1)

                if d < min_d:
                    min_d = d
                    min_f = f  # perpendicular point
            rospy.logdebug("[PoseGeneratorRosInterface.calc_min_distance(PL)] distance: %g" % min_d)
            return min_d, min_f
        else:
            rospy.logwarn("[PoseGeneratorRosInterface.calc_min_distance()] unknown mode %s" % mode)
            return float('NaN')

    @staticmethod
    def _as_pose_stamped(position, floor_msg):
        # type: (np.ndarray, Table) -> PoseStamped
        """
        Generate the target pose for the given position by using the orientation of the plane
        :param position: position [x, y, z]
        :type position: np.ndarray
        :param floor_msg: floor plane
        :type floor_msg: Table
        :return: target pose
        :rtype: PoseStamped
        """
        rospy.logdebug("[PoseGeneratorRosInterface._as_pose_stamped()] given position: %s" % position)
        if not isinstance(position, np.ndarray):
            position = np.asarray(position)
        hull_pnts = []
        for p in floor_msg.convex_hull:
            hull_pnts.append([p.x, p.y, p.z])
        hull_in_floor = PoseGeneratorRosInterface.transform_points_to_plane(hull_pnts, floor_msg)
        assumed_position = position
        if position.shape[0] == 0:
            assumed_position = np.asarray([np.mean(hull_in_floor[:, 0]), 0, 0])
        if position.shape[0] == 1:
            assumed_position = assumed_position + np.hstack([0, np.mean(hull_in_floor[:, 1], 0)])
        if position.shape[0] == 2:
            assumed_position = assumed_position + np.hstack([0, 0, np.mean(hull_in_floor[:, 2])])
        rospy.logdebug("[PoseGeneratorRosInterface._as_pose_stamped()] assumed position: %s" % assumed_position)

        # Xt = PoseGeneratorRosInterface.transform_point_from_plane(assumed_position, floor_msg)
        X = pose_to_array(floor_msg.pose)
        X[:3, 3] = assumed_position
        rospy.logdebug("[PoseGeneratorRosInterface._as_pose_stamped()] X:\n%s" % X)
        ps = PoseStamped()
        ps.header = floor_msg.header
        ps.pose = array_to_pose(X)
        # rospy.loginfo("[PoseGeneratorRosInterface._as_pose_stamped()] ps.pose: %s" % ps)
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
    def metric(d_obstacles, d_hull, wo=1.0):
        """
        Given a distance to the obstacle and a distance to the convex hull, the combined distance is calculated
        :param wo: [optional] weight fot the obstacle distance, default: 1.0
        :type wo: float
        :param d_obstacles: distance to the nearest obstacle
        :type d_obstacles: float
        :param d_hull: distance to the convex hull, perpendicular to the nearest line
        :type d_hull: float
        :return: combined distance
        :rtype: float
        """
        return PoseGeneratorRosInterface.product_metric(d_obstacles, d_hull, wo)

    @staticmethod
    def product_metric(d_obstacles, d_hull, wo=1.0):
        """
        Given a distance to the obstacle and a distance to the convex hull, the combined distance is calculated
        :param wo: [optional] weight fot the obstacle distance, default: 1.0
        :type wo: float
        :param d_obstacles: distance to the nearest obstacle
        :type d_obstacles: float
        :param d_hull: distance to the convex hull, perpendicular to the nearest line
        :type d_hull: float
        :return: combined distance
        :rtype: float
        """
        wh = 1 - wo
        ret_val = np.power(float(d_obstacles), 1 + wo) * np.power(float(d_hull), 1 + wh)
        rospy.logdebug(
            "[PoseGeneratorRosInterface.metric()] %g^%g *%g^%g = %g" % (d_obstacles, wo, d_hull, wh, ret_val))
        return ret_val

    @staticmethod
    def sum_metric(d_obstacles, d_hull, wo=0.5):
        """
        Given a distance to the obstacle and a distance to the convex hull, the combined distance is calculated
        :param wo: [optional] weight fot the obstacle distance, default: 0.5 - weight for hull distance is 1-wo
        :type wo: float
        :param d_obstacles: distance to the nearest obstacle
        :type d_obstacles: float
        :param d_hull: distance to the convex hull, perpendicular to the nearest line
        :type d_hull: float
        :return: combined distance
        :rtype: float
        """
        wh = 1 - wo
        ret_val = float(d_obstacles) * wo + float(d_hull) * wh
        rospy.logdebug(
            "[PoseGeneratorRosInterface.sum_metric()] %g*%g +%g* %g = %g" % (d_obstacles, wo, d_hull, wh, ret_val))
        return ret_val

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
        :param enable_ros: whether or not to use ROS
        :type enable_ros: bool
        """
        self.subsample = sub_sample
        self.ros = enable_ros
        if self.ros:
            from message_filters import Subscriber, Cache
            from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest, GenerateSetPoseResponse
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
        if not self.check_messages(request.obstacles, request.floor):
            rospy.logwarn("[PoseGeneratorRosInterface.handle_service_request()] Messages seem to be faulty")

        from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest, GenerateSetPoseResponse
        response = GenerateSetPoseResponse()
        response.set_pose = self.once(obstacles_msg=request.obstacles, floor_msg=request.floor)
        response.nn_distance, _ = PoseGeneratorRosInterface.calc_min_distance(self.result, self.obs_points, mode="PP")
        response.hull_distance, _ = PoseGeneratorRosInterface.calc_min_distance(self.result, self.hull_points,
                                                                                mode="PL")

        if request.print_evaluation:
            rospy.logwarn("[PoseGeneratorRosInterface.handle_service_request()] print_evaluation is deprecated")

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
        if self.ros and (obstacles_msg is None or floor_msg is None):
            while t is None:
                t = self._obstacle_cache.getLatestTime()
            if obstacles_msg is None:
                while obstacles_msg is None:
                    obstacles_msg = self._obstacle_cache.getElemBeforeTime(t)  # type: MarkerArray
            if floor_msg is None:
                while floor_msg is None:
                    floor_msg = self._floor_cache.getElemBeforeTime(t)  # type: TableArray
                if type(floor_msg) == TableArray:
                    ps = PoseGeneratorRosInterface._as_pose_stamped([], floor_msg.tables[0])
                elif type(floor_msg) is Table:
                    ps = PoseGeneratorRosInterface._as_pose_stamped([], floor_msg)
            ps.header.stamp = t

        if not self.check_messages(obstacles_msg, floor_msg):
            rospy.loginfo("[PoseGeneratorRosInterface.once()] messages not suitable\n%s\n---\n%s" %
                          (obstacles_msg, floor_msg))
            return ps
        if type(floor_msg) == TableArray:
            flr = floor_msg.tables[0]
        else:
            flr = floor_msg
        self.obs_points, self.hull_points = self.extract_points(obstacles_msg, flr)
        # Points are projected into the floor plane, since the normal vector is orthogonal to this plane
        # the z-coordinate for all points within is roughly the same - but not necessary zero
        if len(self.obs_points) == 0:
            # TODO: There are messages with obstacles but something is strange with obstacles_msg.markers[0] when there are more then one
            return ps

        valid_points = PoseGeneratorRosInterface.in_hull(Delaunay(self.hull_points[:, :2], qhull_options="Pp"),
                                                         self.obs_points[:, :2])  # type: np.ndarray
        if len(valid_points) == 0:
            rospy.logdebug("[PoseGeneratorRosInterface.once()] No obstacle points inside the hull")
            # rospy.sleep(0.1)  # next iteration most likely also without obstacles
            self.result = None
        else:
            def calculate_Z(xy, pnts):
                """
                Given an x and y coordinate calculate a z coordinate to lie within a plane defined by a point-set
                :param xy: 2d vector
                :type xy: np.ndarray
                :param pnts: nx3 matrix holding n points, rank should be 2
                :type pnts: np.ndarray
                :return: 3d vector
                :rtype: np.ndarray
                """
                p = pnts[0]
                q = pnts[1]
                l = pnts[2]
                a1 = q[0] - p[0]
                b1 = q[1] - p[1]
                c1 = q[2] - p[2]
                a2 = l[0] - p[0]
                b2 = l[1] - p[1]
                c2 = l[2] - p[2]
                a = b1 * c2 - b2 * c1
                b = a2 * c1 - a1 * c2
                c = a1 * b2 - b1 * a2
                d = (- a * p[0] - b * p[1] - c * p[2])
                z = -1 * (a * xy[0] + b * xy[1] + d) / c
                return np.hstack([xy, z])

            self.result = calculate_Z(
                self._generate(valid_points, hull=self.hull_points[:, :2]), self.hull_points)

        if self.result is None:
            ps.header = flr.header
            ps.pose = flr.pose
        else:
            # result is given in plane coordinates, the transformation in camera coordinates is part of _as_pose_stamped
            ps = PoseGeneratorRosInterface._as_pose_stamped(self.result, flr)

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
        if type(flr_msg) == TableArray and len(flr_msg.tables) == 0:
            rospy.logwarn("[PoseGeneratorRosInterface.check_messages()] No floor plane given")
            return False

        if type(flr_msg) == TableArray:
            flr = flr_msg.tables[0]
        else:
            flr = flr_msg

        if len(flr.convex_hull) < PoseGeneratorRosInterface.HULL_THRESHOLD:
            rospy.logwarn("[PoseGeneratorRosInterface.check_messages()]  Linear hull of the floor is to small (%g/%g)"
                          % (len(flr.convex_hull), PoseGeneratorRosInterface.HULL_THRESHOLD))
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
        # plane_parameters = PoseGeneratorRosInterface.calculate_plane_equation(flr_msg)

        # rospy.loginfo("[PoseGeneratorRosInterface.extract_points()] Obstacles frame: %s" % obs_msg.markers[start].header.frame_id)
        # rospy.loginfo("[PoseGeneratorRosInterface.extract_points()] Floor frame: %s" % flr_msg.header.frame_id)

        i = 0
        if self.subsample >= 1.0:
            ss = self.subsample
        else:
            ss = int(n_obstacles * self.subsample)
        for obstacle in obs_msg.markers[start:]:  # type: Marker
            if ss == 0:
                rospy.logwarn("[PoseGeneratorRosInterface.extract_points] Given %g points and a subsampling rate of %g "
                              "we use %g points - impossible, hence useing at least one" %
                              (len(obstacle.points), self.subsample, len(obstacle.points) * self.subsample))
                point = obstacle.points[len(obstacle.points) % 42]
                obstacles.append([point.x, point.y, point.z])
                continue
            for point in obstacle.points:  # type: Point
                i += 1
                if i % ss == 0:
                    obstacles.append([point.x, point.y, point.z])
        prj_obstacles = PoseGeneratorRosInterface.transform_points_to_plane(obstacles, flr_msg)
        obs_array = np.asarray(obstacles)
        rospy.logdebug("[PoseGeneratorRosInterface.extract_points()] obstacles z coordinate after into plane "
                       "transformation:\n%s\nmean=%g var=%g median=%g" %
                       (obs_array[:, 2], np.mean(obs_array[:, 2]), np.var(obs_array[:, 2]), np.median(obs_array[:, 2])))
        dbg_pnts.extend(prj_obstacles)

        rospy.logdebug(
            "[PoseGeneratorRosInterface.extract_points(%s)] #Markers: %s" % (self.get_name(), len(obstacles)))
        # Add convex hull to dataset
        hull_pnts = []
        for point in flr_msg.convex_hull:
            hull_pnts.append([point.x, point.y, 0])
        prj_hull = PoseGeneratorRosInterface.transform_hull_to_camera(hull_pnts, flr_msg)
        dbg_pnts.extend(prj_hull)

        if self.ros:
            dbg_ma = MarkerArray()
            dbg_mkr = Marker()
            dbg_mkr.header = flr_msg.header
            dbg_mkr.type = Marker.POINTS
            dbg_mkr.action = Marker.ADD
            # POINTS markers use x and y scale for width / height respectively
            dbg_mkr.scale.x = 0.02
            dbg_mkr.scale.y = 0.02
            dbg_mkr.color.r = 0.1216
            dbg_mkr.color.g = 0.4666
            dbg_mkr.color.b = 0.7059
            dbg_mkr.color.a = 1
            dbg_mkr.points = []
            for i_pnt in range(0, len(dbg_pnts)):
                dbg_mkr.points.append(Point(x=dbg_pnts[i_pnt][0], y=dbg_pnts[i_pnt][1], z=dbg_pnts[i_pnt][2]))
            dbg_ma.markers.append(dbg_mkr)
            self._dbg_pub_projection.publish(dbg_ma)

        hull_pnts = np.asarray(hull_pnts)  # np.asarray(hull_pnts)  prj_hull
        obstacles = np.asarray(obstacles)  # np.asarray(obstacles)  prj_obstacles

        hull_pnts = prj_hull
        obstacles = prj_obstacles

        return [obstacles, hull_pnts]

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
            return 'gray'
        elif "roundarea" in name or "oden" in name:
            return 'k'
        elif "elaunay" in name:
            return 'C2'
        elif "inimalDensityEstimate" in name:
            return 'blue'
        elif "onteCarlo" in name or "eferenz" in name:
            return 'C1'
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
        import matplotlib

        # matplotlib.use('TKAgg')
        import matplotlib.pyplot as plt
        from matplotlib import ticker

        if sys.version_info.major < 3:
            # see: https://stackoverflow.com/questions/55554352/import-of-matplotlib2tikz-results-in-syntaxerror-invalid-syntax
            import matplotlib2tikz as mpl2tkz
        else:
            import tikzplotlib as mpl2tkz

        if ax is None:
            fig, ax = plt.subplots()
        else:
            fig = plt.gcf()
        n = self.get_name()
        c = PoseGeneratorRosView.get_color(n)
        # TODO: Fix matplotlib deprecated warning
        if type(self.result) is list or (len(self.result.shape) == 1 and self.result.shape[0] == 3):
            self.result = np.asarray([self.result])
        for lne in self.lines:
            x = [lne[0][0], lne[1][0]]
            y = [lne[0][1], lne[1][1]]
            # ax.plot([lne[0][0], lne[1][0]], [lne[0][1], lne[1][1]], ':', color=c, alpha=0.75, zorder=5,
            ax.plot(x, y, linestyle='dashed', color=c, alpha=0.75, zorder=5, label=n + " Hilfslinien")
        # try:
        #     # https://tikzplotlib.readthedocs.io/en/latest/index.html#tikzplotlib.clean_figure
        #     mpl2tkz.clean_figure(fig)
        # except AttributeError as ae:
        #     rospy.logerr("[view_all()] AttributeError at <mpl2tkz.clean_figure(fig)> %s" % ae)
        # except ValueError as ve:
        #     rospy.logerr("[view_all()] ValueError at <mpl2tkz.clean_figure(fig)> %s" % ve)

        if obstacles:
            n = "Hindernisse"
            ax.plot(self.obs_points[:, 0], self.obs_points[:, 1], '.', label=n, zorder=3.5,
                    color=PoseGeneratorRosView.get_color(n))
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

    def get_name(self):
        return u'PCA'

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
        rospy.logdebug("[PcaPoseGenerator._generate()] p: %s  " % p)
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
        super(MinimalDensityEstimatePoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample,
                                                                  enable_ros)
        self.n_bins = int(n_bins)  # type: int
        # self.hlp_xx, self.hlp_yy = np.mgrid[-1:1: self.n_bins * 1j, -1:1: self.n_bins * 1j]
        self.hlp_positions = np.zeros((self.n_bins ** 2, 2))
        self.hlp_f = np.zeros(self.n_bins ** 2)

        # rospy.loginfo("[MinimalDensityEstimatePoseGenerator.__init__()] self.hlp_positions %s" % self.hlp_positions)
        # rospy.loginfo("[MinimalDensityEstimatePoseGenerator.__init__()]  self.hlp_f %s" % self.hlp_f)

    def get_name(self):
        return u'KDE'

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

        # Perform the kernel density estimate
        xx, yy = np.mgrid[xmin:xmax: self.n_bins * 1j, ymin:ymax: self.n_bins * 1j]
        all_positions = np.vstack([xx.ravel(), yy.ravel()]).T
        positions = PoseGeneratorRosInterface.in_hull(Delaunay(hull[:, :2], qhull_options="Pp"), all_positions)
        values = np.vstack([x, y])
        kernel = st.gaussian_kde(values)

        self.hlp_f = kernel(all_positions.T)
        self.hlp_positions = all_positions

        z_i = np.argmin(kernel(positions.T))
        z_x = positions[z_i, 0]
        z_y = positions[z_i, 1]

        return np.asarray([z_x, z_y])

    # def plot(self, ax, hull=True, obstacles=True):
    #     """
    #     Plot the information of this PoseGenerator into an outside existing matplotlib figure
    #     :param hull: plot the hull
    #     :type hull: bool
    #     :param obstacles: plot the obstacles
    #     :type obstacles: bool
    #     :param ax: axis handle
    #     :type ax: plt.axes.Axis
    #     :return: data point axis
    #     :rtype: plt.axes.Axis
    #     """
    #     ret_ax = super(MinimalDensityEstimatePoseGenerator, self).plot(ax, hull, obstacles)
    #     l = int(np.sqrt(len(self.hlp_f)))
    #     xmin, xmax = min(self.hlp_positions[:, 0]), max(self.hlp_positions[:, 0])
    #     ymin, ymax = min(self.hlp_positions[:, 1]), max(self.hlp_positions[:, 1])
    #     if xmin == xmax or ymin == ymax:
    #         rospy.logdebug("[MinimalDensityEstimatePoseGenerator.plot()] xmin: %s" % xmin)
    #         rospy.logdebug("[MinimalDensityEstimatePoseGenerator.plot()] xmax: %s" % xmax)
    #         rospy.logdebug("[MinimalDensityEstimatePoseGenerator.plot()] ymin: %s" % ymin)
    #         rospy.logdebug("[MinimalDensityEstimatePoseGenerator.plot()] ymax: %s" % ymax)
    #         return
    #     xx, yy = np.mgrid[xmin:xmax:l * 1j, ymin:ymax:l * 1j]
    #     f = np.reshape(self.hlp_f[:l ** 2].T, xx.shape)
    #     cs = ax.contourf(xx, yy, f, cmap='Blues', alpha=0.5, zorder=1, locator=ticker.LogLocator())
    #     plt.gcf().colorbar(cs)
    #     cset = ax.contour(xx, yy, f, colors=PoseGeneratorRosView.get_color(self.get_name()), alpha=0.2, zorder=2)
    #     ax.clabel(cset, inline=True, fontsize=10, fmt='%1.2f')  # , zorder=3
    #
    #     return ret_ax


class MonteCarloPoseGenerator(PoseGeneratorRosView):
    """
    Discretion of the search area and run nn approach for hull and obstacles
    """

    def __init__(self, pub_topic=DF_PUB_TOPIC, obs_topic=DF_OBS_TOPIC, flr_topic=DF_FLR_TOPIC, sub_sample=DF_SUB_SAMPLE,
                 enable_ros=DF_ENABLE_ROS, mc_raster=DF_MC_RASTER, mc_wo=DF_MC_WO):
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
        :param mc_wo: weight for distance to obstacle
        :type mc_wo: float
        """
        super(MonteCarloPoseGenerator, self).__init__(pub_topic, obs_topic, flr_topic, sub_sample, enable_ros)
        self.n_xlines = mc_raster
        self.n_ylines = self.n_xlines
        self.wo = mc_wo

    def get_name(self):
        return "Referenz"

    def _generate(self, lst_obs_points, hull=None):
        """
        Algorithm to determine a valid pose given a floor plane and obstacles
        :param lst_obs_points: list with all obstacle points projected on the floor plane
        :type lst_obs_points: np.ndarray
        :param hull: convex hull for the dataset
        :type hull: list
        :return: tuple of the position [x, y, z]
        :rtype: np.ndarray
        """
        # see: https://stackoverflow.com/questions/41577705/how-does-2d-kernel-density-estimation-in-python-sklearn-work
        if hull is None:
            hull = []
        hull = np.asarray(hull)

        # see: https://stackoverflow.com/questions/30145957/plotting-2d-kernel-density-estimation-with-python
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
        # hul_nn = NearestNeighbors(n_neighbors=1, algorithm='auto', metric='euclidean').fit(
        #     hull)  # type: NearestNeighbors
        distances, indices = obs_nn.kneighbors(positions)
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] distances = %s" % distances)
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] indices = %s" % indices)

        # Distance calculation
        extended_positions = np.ndarray((len(positions), 3))
        for i in range(0, len(positions)):
            hull_distance, _ = PoseGeneratorRosInterface.calc_min_distance(positions[i], hull.tolist(), mode="PL")
            d, i_dp = obs_nn.kneighbors([positions[i, 0:2]])
            m = PoseGeneratorRosInterface.metric(d_obstacles=d[0], d_hull=hull_distance, wo=self.wo)
            extended_positions[i] = np.append(positions[i], m)
        i_max = np.argmax(extended_positions[:, -1])
        ret_pos = extended_positions[i_max, 0:-1]
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] position %s at index %g with distance %g" %
                       (ret_pos, i_max, extended_positions[i_max, -1]))
        rospy.logdebug("[MonteCarloPoseGenerator._generate()] distances: \n%s" % extended_positions[:, -1])
        # Calculate help lines
        obs_d, obs_i = obs_nn.kneighbors([ret_pos])
        hul_d, hp = PoseGeneratorRosInterface.calc_min_distance(ret_pos, hull.tolist(), mode="PL")
        obs_i = obs_i[0][0]
        obs = lst_obs_points[obs_i]

        self.lines = [[hp, ret_pos], [ret_pos, obs]]

        return ret_pos
