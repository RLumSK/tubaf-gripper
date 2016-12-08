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
import message_filters
import tf

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from object_recognition_msgs.msg import TableArray

class ObjectFilter(object):
    """
    Class is used in conjunction with the tabletop pipeline of the object recognition kitchen (ORK) framework, see: http://wg-perception.github.io/tabletop/index.html#tabletop
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
        _pose_topic = rospy.get_param('~pose_topic', "/ork/obj_poses")
        _floor_topic = rospy.get_param('~floor_topic', "/ork/floor_plane")
        self.threshold = rospy.get_param('threshold_paramter', 0.9)
        # publisher
        self._pose_publisher = rospy.Publisher(name=_pose_topic, data_class=PoseArray, queue_size=10)
        self._cluster_publisher = rospy.Publisher(name=_filtered_clusters_topic, data_class=MarkerArray, queue_size=10)
        # subscriber
        rospy.Subscriber(name=_cluster_topic, data_class=MarkerArray, callback=self._on_new_cluster)
        _sub_floor = message_filters.Subscriber(_floor_topic, TableArray)
        self.floor_cache = message_filters.Cache(_sub_floor, 10)
        self._tf = tf.TransformListener()

    def _on_new_cluster(self, msg):
        """
        Callback for visualization_msgs/MarkerArray message, those clusters are analysed and objects are searched within
        :param msg: list of clusters
        :type msg: visualization_msgs.msgs.MarkerArray
        :return: -
        :rtype: None
        """
        lst_cluster = msg.markers

        # identify floor
        msg_floor = self.floor_cache.getElemAfterTime(self.floor_cache.getLastestTime())
        if msg_floor is None:
            return
        rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster] TableArray has %d planes stored", len(msg_floor.tables))
        floor_plane = msg_floor.tables[0]

        # analyse given clusters
        lst_sizes = []
        for cluster in lst_cluster:
            lst_sizes.append(len(cluster.points))
        _min = min(lst_sizes)
        _max = max(lst_sizes)
        _threshold = _min+self.threshold*(_max - _min)

        # identify object clusters
        lst_obj_cluster = []
        for cluster in lst_cluster:
            # cluster is of type: visualization_msgs/Marker (http://docs.ros.org/indigo/api/visualization_msgs/html/msg/Marker.html)
            rospy.logdebug("[cluster_analysis::ObjectFilter._on_new_cluster] cluster["+str(cluster.id)+"] has "+str(len(cluster.points))+" points")
            if len(cluster.points) > _threshold:
                lst_obj_cluster.append(cluster)

        if len(lst_obj_cluster) == 0:
            return

        # publish results
        _obj_cluster_msg = MarkerArray()
        _pose_array = PoseArray()
        _pose_array.header = msg_floor.header
        for cluster in lst_obj_cluster:
            _pose_array.poses.append(self.generate_pose(cluster, floor_plane))
            _obj_cluster_msg.markers.append(cluster)
        self._pose_publisher.publish(_pose_array)
        self._cluster_publisher.publish(_obj_cluster_msg)

        # debugging
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] floor_plane frame_id:\n" +
        #               str(floor_msg.tables[0].header.frame_id))
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] floor_plane orientation:\n" +
        #               str(floor_msg.tables[0].pose.orientation))
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] obj_pose frame_id:\n" +
        #               str(ps.header.frame_id))
        # rospy.loginfo("[cluster_analysis::ObjectFilter._on_new_cluster] obj_pose orientation:\n" +
        #               str(ps.pose.orientation))

    def transform(self, frame_from, frame_to, pose):
        """
        Transform a given pose into another frame using the transform library (tf)
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
        if self._tf.frameExists(frame_from) and self._tf.frameExists(frame_to):
            ps = PoseStamped()
            ps.header.frame_id = frame_from
            ps.pose = pose
            # t = self._tf.getLatestCommonTime(frame_from, frame_to)
            # position, quaternion = self._tf.lookupTransform(frame_from, frame_to, t)
            # print position, quaternion
            self._tf.waitForTransform(frame_from, frame_to, rospy.Time(), rospy.Duration(4.0))
            try:
                ret_ps = self._tf.transformPose(frame_to, ps)
            except Exception as ex:
                rospy.logwarn(
                    "[cluster_analysis::ObjectFilter.transform] Couldn't Transform from " + frame_from + " to " + frame_to)
                rospy.logwarn(ex.message)
            return ret_ps.pose
        rospy.logwarn("[cluster_analysis::ObjectFilter.transform] Couldn't Transform from "+frame_from+" to "+frame_to)
        return None

    def generate_pose(self, points, plane):
        """
        Generate a ROS pose from a given set of points - in the context of the package typical a cluster the orientation
        of the floor plane is used as orientation for the pose
        :param points: Points
        :type points: visualization_msgs.msg.Marker
        :param plane: Floor plane
        :type plane: object_recognition_msgs.msgs.Table
        :return: Pose of the cluster - mean
        :rtype: geometry_msgs.msg.Pose
        """
        ret_pose = Pose()
        x_lst = []
        y_lst = []
        z_lst = []
        for point in points.points:
            x_lst.append(point.x)
            y_lst.append(point.y)
            z_lst.append(point.z)
        ret_pose.position.x = np.mean(x_lst)
        ret_pose.position.y = np.mean(y_lst)
        ret_pose.position.z = np.mean(z_lst)
        if plane is not None:
            rospy.logdebug("[cluster_analysis::ObjectFilter.generate_pose] plane-frame=:"+str(plane.header.frame_id))
            rospy.logdebug("[cluster_analysis::ObjectFilter.generate_pose] points-frame=:"+str(points.header.frame_id))
            rospy.logdebug("[cluster_analysis::ObjectFilter.generate_pose] before:\n"+str(plane.pose.orientation))
            pose = self.transform(plane.header.frame_id, points.header.frame_id, plane.pose)
            ret_pose.orientation = pose.orientation
            # ret_pose.orientation = points.pose.orientation
            rospy.logdebug("[cluster_analysis::ObjectFilter.generate_pose] after:\n"+str(ret_pose.orientation))
        return ret_pose


if __name__ == '__main__':
    rospy.init_node("object_filter")
    obj = ObjectFilter()
    rospy.spin()
