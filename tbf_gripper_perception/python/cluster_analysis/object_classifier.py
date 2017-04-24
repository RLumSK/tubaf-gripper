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
from object_recognition_msgs.msg import TableArray

from tbf_gripper_perception.srv import ObjectClassifierService
from tbf_gripper_perception.msg import DetectedObject


class ObjectClassifier(object):
    """
    Class is used to specify the object based on a given cluster of points.
    The class aims to select a mesh or matching primitive for the object and determine its reference pose.
    The class implements a ROS Service - http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
    """

    def __init__(self):
        """
        Default-Constructor - TODO Description
        """
        # Server
        rospy.init_node("object_classifier")
        self.service = rospy.Service("object_classifier", ObjectClassifierService, self.get_objects)

        # Parameter
        _filtered_cloud_topic = rospy.get_param('~filtered_cloud_topic', "/ork/obj_cloud")
        _floor_topic = rospy.get_param('~floor_topic', "/ork/floor_plane")
        self.threshold = rospy.get_param('threshold_parameter', 0.9)

        # Subscriber
        rospy.Subscriber(_filtered_cloud_topic, MarkerArray, self._on_new_clusters)
        # rospy.Publisher(name=_filtered_cloud_topic, data_class=PointCloud2, queue_size=10)
        # Publisher

        # Cache, TF
        _floor_msgsub = message_filters.Subscriber(_floor_topic, TableArray)
        self.floor_cache = message_filters.Cache(_floor_msgsub, 1)
        self._tf = tf.TransformListener()
        ###########
        self.lst_objects = []

    def _on_new_clusters(self, msg):
        """
        Callback from the perception node with detected object clusters
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
            rospy.logdebug("[cluster_analysis::ObjectClassifier._on_new_cluster] cluster[" + str(cluster.id) + "] has " +
                           str(len(cluster.points)) + " points")
            if len(cluster.points) >= _threshold:
                lst_obj_cluster.append(cluster)

        if len(lst_obj_cluster) == 0:
            rospy.logdebug("[cluster_analysis::ObjectClassifier._on_new_cluster] cluster don't has enough points by a "
                           "threshold of" + str(_threshold))
            return

        # get the cluster farthest away from the floor plane
        floor = self.floor_cache.getElemBeforeTime(rospy.Time.now())
        if floor is None:
            rospy.logwarn("cluster_analysis::ObjectClassifier._on_new_cluster(): no floor published")
            return
        ps_floor = PoseStamped(floor.header, floor.tables[0].pose)
        rospy.logdebug("[cluster_analysis::ObjectClassifier._on_new_cluster]: got " + str(len(lst_obj_cluster)) +
                       " clusters")
        for cluster in lst_obj_cluster:
            ps_cluster = generate_pose(self._tf, cluster)
            cluster_pose = transform(self._tf, ps_floor.header.frame_id, ps_cluster.header.frame_id, ps_cluster.pose)
            self._update_obj_list(cluster, cluster_pose)

    def _update_obj_list(self, cluster, pose):
        """
        Manage the objects list by adding, updating, ... based on the given cluster and its pose
        :param cluster: cluster of points that partial describes an object
        :type cluster: visualization_msgs.msg.Marker
        :param pose: pose of the cluster
        :type pose: geometry_msgs.msg.Pose
        :return:
        """
        #TODO object classification

    def get_objects(self):
        """
        This is the service interface, when called return a list with the currently known objects
        :return: list with tbf_gripper_perception.DetectedObject.msg's
        """
        return self.lst_objects

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
    _tmp = transform(tf, "base_link", points.header.frame_id, base_link_pose)
    ret_pose.pose.orientation = _tmp.orientation
    ret_pose.header = points.header

    return ret_pose


if __name__ == "__main__":
    classifier = ObjectClassifier()
    rospy.spin()
