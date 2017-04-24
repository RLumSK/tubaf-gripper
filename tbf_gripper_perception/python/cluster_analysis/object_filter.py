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
import numpy as np
import tf
import tf.transformations
import message_filters

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from numpy import linalg as LA
from object_recognition_msgs.msg import TableArray
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
import sensor_msgs.point_cloud2 as pc_2


class ObjectFilter(object):
    """
    Class is used in conjunction with the tabletop pipeline of the object recognition kitchen (ORK) framework,
      see: http://wg-perception.github.io/tabletop/index.html#tabletop
    to identify objects from given clusters. You can think of it as a filter that tries to identify clusters that are
    actual objects and not artifacts from any algorithms or occlusions.
    """

    def __init__(self):
        """
        Default-Constructor - initializing subscriber and publisher and getting parameter
        """
        # parameter
        _cluster_topic = rospy.get_param('~cluster_topic', "/ork/tabletop/clusters")
        _filtered_clusters_topic = rospy.get_param('~filtered_clusters_topic', "/ork/obj_clusters")
        _filtered_cloud_topic = rospy.get_param('~filtered_cloud_topic', "/ork/obj_cloud")
        _pose_topic = rospy.get_param('~pose_topic', "/ork/obj_poses")
        _floor_topic = rospy.get_param('~floor_topic', "/ork/floor_plane")
        _planning_scene_topic = rospy.get_param('~planning_scene_topic', "/julius_moveit/planning_scene")
        self.threshold = rospy.get_param('threshold_parameter', 0.9)
        # publisher
        self._pose_publisher = rospy.Publisher(name=_pose_topic, data_class=PoseArray, queue_size=10)
        self._cluster_publisher = rospy.Publisher(name=_filtered_clusters_topic, data_class=MarkerArray, queue_size=10)
        self._pointcloud_publisher = rospy.Publisher(name=_filtered_cloud_topic, data_class=PointCloud2, queue_size=10)
        self._planning_scene_diff_publisher = rospy.Publisher(name=_planning_scene_topic, data_class=PlanningScene,
                                                              queue_size=1)
        # subscriber
        self.marker_array_subscriber = rospy.Subscriber(name=_cluster_topic, data_class=MarkerArray,
                                                        callback=self._on_new_cluster)
        self._tf = tf.TransformListener()
        # cache
        _floor_msgsub = message_filters.Subscriber(_floor_topic, TableArray)
        self.floor_cache = message_filters.Cache(_floor_msgsub, 1)
        # object
        _primitive = SolidPrimitive()
        _primitive.type = SolidPrimitive.SPHERE
        _primitive.dimensions.append(0.2)
        self.co_wlan_station = CollisionObject()
        self.co_wlan_station.id = "wlan_box"
        # TODO determine and add mesh of the object here
        self.co_wlan_station.operation = CollisionObject.ADD
        self.co_wlan_station.primitives.append(_primitive)

    def _on_new_cluster(self, msg):
        """
        Callback for visualization_msgs/MarkerArray message, those clusters are analysed and objects are searched within
        :param msg: list of clusters
        :type msg: visualization_msgs.msgs.MarkerArray
        :return: -
        :rtype: None
        """
        lst_cluster = msg.markers
        # analyse given clusters
        lst_sizes = []
        for cluster in lst_cluster:
            lst_sizes.append(len(cluster.points))
        _min = min(lst_sizes)
        _max = max(lst_sizes)
        _threshold = _min + self.threshold * (_max - _min)

        # identify object clusters
        lst_obj_cluster = []
        for cluster in lst_cluster:
            # cluster is of type: visualization_msgs/Marker
            # (http://docs.ros.org/indigo/api/visualization_msgs/html/msg/Marker.html)
            rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster] cluster[" + str(cluster.id) + "] has " +
                           str(len(cluster.points)) + " points")
            if len(cluster.points) >= _threshold:
                lst_obj_cluster.append(cluster)

        if len(lst_obj_cluster) == 0:
            rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster] cluster don't has enough points by a "
                           "threshold of" + str(_threshold))
            return

        # get the cluster farthest away from the floor plane
        floor = self.floor_cache.getElemBeforeTime(rospy.Time.now())
        if floor is None:
            rospy.logwarn("cluster_analysis::ObjectFilter._on_new_cluster(): no floor published")
            return
        ps_floor = PoseStamped(floor.header, floor.tables[0].pose)
        max_height = float("-inf")
        selected_cluster = None
        selected_pose = None
        rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster]: got " + str(len(lst_obj_cluster)) +
                       " clusters")
        for cluster in lst_obj_cluster:
            ps_cluster = self.generate_pose(cluster)
            cluster_pose = self.transform(ps_floor.header.frame_id, ps_cluster.header.frame_id, ps_cluster.pose)
            height = cluster_pose.position.z - ps_floor.pose.position.z
            # rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster]: height=" + str(height))
            if(height > max_height):
                selected_pose = ps_cluster.pose
                selected_cluster = cluster
                max_height = height
                # rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster] cluster[" + str(cluster.id) + "] "
                #              "has a distance of " + str(max_height))
        # rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster]: got " + str(selected_cluster))

        # publish results
        _obj_cluster_msg = MarkerArray()
        _pose_array = PoseArray()
        _pose_array.header = lst_cluster[0].header
        # for cluster in lst_obj_cluster:
        #     _pose_array.poses.append(self.generate_pose(cluster))
        #     # _pose_array.poses.append(self.generate_pose_pca(cluster))
        #     _obj_cluster_msg.markers.append(cluster)
        if selected_cluster is None or selected_pose is None:
            return
        _pose_array.poses.append(selected_pose)
        _obj_cluster_msg.markers.append(selected_cluster)
        self._pose_publisher.publish(_pose_array)
        self._cluster_publisher.publish(_obj_cluster_msg)
        self._pointcloud_publisher.publish(ObjectFilter.cluster_to_pointcloud(cluster=selected_cluster))

        # debugging
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] floor_plane frame_id:\n" +
        #               str(floor_msg.tables[0].header.frame_id))
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] floor_plane orientation:\n" +
        #               str(floor_msg.tables[0].pose.orientation))
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] obj_pose frame_id:\n" +
        #               str(ps.header.frame_id))
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] obj_pose orientation:\n" +
        #               str(ps.pose.orientation))

        # publish object
        _scene = PlanningScene()
        self.co_wlan_station.header = _pose_array.header
        if len(self.co_wlan_station.primitive_poses) == 0:
            self.co_wlan_station.primitive_poses.append(selected_pose)
        else:
            self.co_wlan_station.primitive_poses[0] = selected_pose
        _scene.is_diff = True
        _scene.world.collision_objects.append(self.co_wlan_station)
        self._planning_scene_diff_publisher.publish(_scene)
        rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster] published update to planning_scene")
        # TODO add also transform between "_object.link_name" o-o "_pose_array.header.frame_id" ???

    @staticmethod
    def transform(tf, frame_from, frame_to, pose):
        """
        Transform a given pose into another frame using the transform library (tf)
        :param tf: transformation object
        :type tf: TransfromListener
        :param frame_from: frame in which the orientation is valid
        :type frame_from: string
        :param frame_to:target frame for the transformation
        :type frame_to: string
        :param pose: pose that should be transformed
        :type pose: geometry_msgs.msg.Pose
        :return: transformed pose
        :rtype: geometry_msgs.msg.Pose
        """
        # see:  http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
        #       http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20(Python)
        if tf.frameExists(frame_from) and tf.frameExists(frame_to):
            ps = PoseStamped()
            ps.header.frame_id = frame_from
            ps.pose = pose
            # t = self._tf.getLatestCommonTime(frame_from, frame_to)
            # position, quaternion = self._tf.lookupTransform(frame_from, frame_to, t)
            # print position, quaternion
            tf.waitForTransform(frame_from, frame_to, rospy.Time(), rospy.Duration(4))
            try:
                ret_ps = tf.transformPose(frame_to, ps)
                return ret_ps.pose
            except Exception as ex:
                rospy.logwarn("[cluster_analysis::ObjectFilter.transform] Couldn't Transform from " + frame_from +
                              " to " + frame_to)
                rospy.logwarn(ex.message)
        rospy.logwarn(
            "[cluster_analysis::ObjectFilter.transform] Couldn't Transform from " + frame_from + " to " + frame_to)
        return None

    @staticmethod
    def generate_pose(tf, points):
        """
        Generate a ROS pose from a given set of points - in the context of the package typical a cluster the orientation
        of the base_link is used as orientation for the pose
        :param tf:transformation object
        :type tf: TransfromListener
        :param points: Points
        :type points: visualization_msgs.msg.Marker
        :return: Pose of the cluster - mean
        :rtype: geometry_msgs.msg.PoseStamped
        """
        ret_pose = PoseStamped()
        x_lst = []
        y_lst = []
        z_lst = []
        for point in points.points:
            x_lst.append(point.x)
            y_lst.append(point.y)
            z_lst.append(point.z)
        ret_pose.pose.position.x = np.mean(x_lst)
        ret_pose.pose.position.y = np.mean(y_lst)
        ret_pose.pose.position.z = np.mean(z_lst)

        a = 0
        c = 0
        b = 1
        d = 1
        Z = np.sqrt(a**2+b**2+c**2+d**2)
        w_05 = np.sqrt(0.5)
        base_link_pose = Pose()
        base_link_pose.position.x = 0
        base_link_pose.position.y = 0
        base_link_pose.position.z = 0
        base_link_pose.orientation.x = b/Z
        base_link_pose.orientation.y = c/Z
        base_link_pose.orientation.z = d/Z
        base_link_pose.orientation.w = a/Z
        _tmp = ObjectFilter.transform(tf, "base_link", points.header.frame_id, base_link_pose)
        ret_pose.pose.orientation = _tmp.orientation
        ret_pose.header = points.header

        return ret_pose

    def generate_pose_pca(self, points):
        """
        NOT WORKING
        Generate a ROS pose from a given set of points - a principal component analysis is used to determine the center
        and orientation of the object. first axis = x, second axis =y, third axis = z
        :param points: cluster of points
        :type points: visualization_msgs.msg.Marker
        :return: Pose of the cluster
        :rtype: geometry_msgs.msg.Pose
        """
        ret_pose = Pose()
        _point_lst = []
        for point in points.points:
            _point_lst.append([point.x, point.y, point.z])
        _points_matrix = np.asmatrix(_point_lst)
        # http://stackoverflow.com/questions/13224362/principal-component-analysis-pca-in-python
        #
        m, n = _points_matrix.shape  # m = #points, n = #dimensions
        rospy.loginfo("[cluster_analysis::ObjectFilter.generate_pose_pca] #Points = " + str(m))
        rospy.loginfo("[cluster_analysis::ObjectFilter.generate_pose_pca] #Dimensions = " + str(n))

        # mean center the data
        data = _points_matrix - _points_matrix.mean(axis=0)
        # calculate the covariance matrix
        R = np.cov(data)
        rospy.loginfo("[cluster_analysis::ObjectFilter.generate_pose_pca] cov = " + str(R))
        # calculate eigenvectors & eigenvalues of the covariance matrix
        # use 'eigh' rather than 'eig' since R is symmetric,
        # the performance gain is substantial
        evals, evecs = LA.eigh(R)
        # sort eigenvalue in decreasing order
        idx = np.argsort(evals)[::-1]
        evecs = evecs[:, idx]
        # sort eigenvectors according to same index
        # evals = evals[idx]
        # select the first n eigenvectors (n is desired dimension
        # of rescaled data array, or dims_rescaled_data)
        evecs = evecs[:, :]
        # data_pca = np.dot(evecs.T, data.T).T

        rospy.loginfo("[cluster_analysis::ObjectFilter.generate_pose_pca] evecs = " + str(evecs))
        X = np.asmatrix(evecs)
        X_0 = np.asmatrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        Rot = X*np.invert(X_0)

        _tmp = _points_matrix.mean(axis=0)[0]
        rospy.loginfo("[cluster_analysis::ObjectFilter.generate_pose_pca] Rot = " + str(Rot))
        _quat = tf.transformations.quaternion_from_matrix(Rot)
        rospy.loginfo("[cluster_analysis::ObjectFilter.generate_pose_pca] Quaternion = " + str(_quat))
        ret_pose.position.x = _tmp[0]
        ret_pose.position.y = _tmp[1]
        ret_pose.position.z = _tmp[2]
        ret_pose.orientation = _quat

        return ret_pose

    @staticmethod
    def cluster_to_pointcloud(cluster):
        """
        Convert a Cluster of points represented as Markers into a PointCloud
        :param cluster: cluster of points
        :type cluster: visualization_msgs.msg.Marker
        :return:
        """
        lst = []
        for point in cluster.points:
            lst.append([point.x, point.y, point.z])
        return pc_2.create_cloud_xyz32(header=cluster.header, points=lst)

if __name__ == '__main__':
    rospy.init_node("object_filter", log_level=rospy.INFO)
    obj = ObjectFilter()
    rospy.spin()
