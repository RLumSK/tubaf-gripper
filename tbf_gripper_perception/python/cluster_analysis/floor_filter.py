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
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, QuaternionStamped
from object_recognition_msgs.msg import TableArray, Table


class FloorFilter(object):
    """
    Class is used in conjunction with the tabletop pipeline of the object recognition kitchen (ORK) framework, see: http://wg-perception.github.io/tabletop/index.html#tabletop
    to identify the floor plane on which the robot currently stands
    """

    def __init__(self):
        """
        Default-Constructor - initializing subscriber and publisher and getting parameter
        """
        # parameter
        _tables_topic = rospy.get_param('~tables_topic', "/ork/table_array")
        _floor_topic = rospy.get_param('~floor_topic', "/ork/floor_plane")
        self.floor_frame = rospy.get_param('~floor_frame', "base_link")
        self.floor_frame_offset = rospy.get_param('~floor_frame_offset', 0.53)
        # publisher
        self._floor_publisher = rospy.Publisher(name=_floor_topic, data_class=TableArray, queue_size=10)
        # subscriber
        rospy.Subscriber(name=_tables_topic, data_class=TableArray, callback=self._on_new_tables)
        self._tf = tf.TransformListener()

    def _on_new_tables(self, msg):
        """
        Callback for object_recognition_msgs/TableArray message, those planes are analysed and the floor is identified
        :param msg: list of planes
        :type msg: object_recognition_msgs.msgs.TableArray
        :return: -
        :rtype: None
        """
        rospy.logdebug("[cluster_analysis::FloorFilter._on_new_tables] TableArray has %d planes stored", len(msg.tables))
        floor_plane = self.identify_floor(msg)
        if floor_plane is not None:
            floor_msg = TableArray()
            floor_msg.header = floor_plane.header
            floor_msg.tables.append(floor_plane)
            self._floor_publisher.publish(floor_msg)
        else:
            rospy.logwarn("[cluster_analysis::FloorFilter._on_new_tables] No floor plane found")

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
            self._tf.waitForTransform(frame_from, frame_to, rospy.Time(), rospy.Duration(4.0))
            try:
                ret_ps = self._tf.transformPose(frame_to, ps)
            except Exception as ex:
                rospy.logwarn(
                    "[cluster_analysis::FloorFilter.transform] Couldn't Transform from " + frame_from + " to " + frame_to)
                rospy.logwarn(ex.message)
            return ret_ps.pose
        rospy.logwarn("[cluster_analysis::FloorFilter.transform] Couldn't Transform from "+frame_from+" to "+frame_to)
        return None

    def identify_floor(self, tables):
        """
        Identify the floor plane from a given list
        :param tables: list of planes
        :type tables: object_recognition_msgs.msgs.TableArray
        :return: floor plane
        :rtype: object_recognition_msgs.msgs.Table
        """
        ret_plane = None
        lst_ret_planes = []
        # Determine planes with a parallel normal vector
        for plane in tables.tables:
            pose = self.transform(plane.header.frame_id, self.floor_frame, plane.pose)
            if pose is None:
                continue
            rospy.logdebug("[cluster_analysis::ObjectFilter.identify_floor] plane-frame_id = " + plane.header.frame_id)
            rospy.logdebug("[cluster_analysis::ObjectFilter.identify_floor] floor-frame_id = " + self.floor_frame)
            rospy.logdebug("[cluster_analysis::ObjectFilter.identify_floor]  its quaternion = (" +
                          str(pose.orientation.x)+"," + str(pose.orientation.y)+"," +
                          str(pose.orientation.z)+"," + str(pose.orientation.w)+")")
            if abs(pose.orientation.z) > (2*(abs(pose.orientation.x) + abs(pose.orientation.y))):
                lst_ret_planes.append(plane)
        rospy.logdebug("[cluster_analysis::ObjectFilter.identify_floor] found %d planes that qualify",len(lst_ret_planes))
        # Determine closest plane to the floor (z=0 @ base_link, floor is approx. 53 cm below base_link)
        min_z = float("inf")
        for plane in lst_ret_planes:
            rospy.logdebug("[cluster_analysis::ObjectFilter.identify_floor] pose = " + str(plane.pose))
            if min_z > abs(plane.pose.position.z+self.floor_frame_offset):
                ret_plane = plane
                min_z = plane.pose.position.z
        return ret_plane


if __name__ == '__main__':
    rospy.init_node("floor_filter")
    obj = FloorFilter()
    rospy.spin()
