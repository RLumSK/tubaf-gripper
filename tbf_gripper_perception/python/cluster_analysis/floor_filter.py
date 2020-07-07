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

import sys
import signal
import rospy
import tf

from geometry_msgs.msg import PoseStamped, QuaternionStamped
from object_recognition_msgs.msg import TableArray
from message_filters import Subscriber, Cache


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class FloorFilter(object):
    """
    Class is used in conjunction with the tabletop pipeline of the object recognition kitchen (ORK) framework,
      see: http://wg-perception.github.io/tabletop/index.html#tabletop
    to identify the floor plane on which the robot currently stands
    """

    def __init__(self):
        """
        Default-Constructor - initializing subscriber and publisher and getting parameter
        """
        # parameter
        _tables_topic = rospy.get_param('~tables_topic', "/ork/table_array")
        _floor_topic = rospy.get_param('~floor_topic', "/ork/floor_plane")
        _normal_topic = rospy.get_param('~normal_topic', "/ork/floor_normal")
        _pose_topic = rospy.get_param('~pose_topic', "/ork/floor_pose")
        self.floor_frame = rospy.get_param('~floor_frame', "base_footprint")
        self.floor_frame_offset = rospy.get_param('~floor_frame_offset', 0.53)
        # publisher
        self._floor_publisher = rospy.Publisher(name=_floor_topic, data_class=TableArray, queue_size=10)
        self._normal_publisher = rospy.Publisher(name=_normal_topic, data_class=QuaternionStamped, queue_size=10)
        self._pose_publisher = rospy.Publisher(name=_pose_topic, data_class=PoseStamped, queue_size=10)
        # Cache
        self._planes_cache = Cache(Subscriber(_tables_topic, TableArray), 1)
        self._tf = tf.TransformListener()

    def perform(self):
        """
        Main Loop handling object_recognition_msgs/TableArray message, those planes are analysed
        and the floor is identified
        :return: -
        :rtype: None
        """
        while not rospy.is_shutdown():
            msg = self._planes_cache.getLast()
            if msg is None:
                rospy.sleep(1.0)
                continue
            floor_plane = self.identify_floor(msg)
            if floor_plane is not None:
                floor_msg = TableArray()
                floor_msg.header = floor_plane.header
                floor_msg.tables.append(floor_plane)
                self._floor_publisher.publish(floor_msg)
                qs_msg = QuaternionStamped()
                qs_msg.header = floor_plane.header
                qs_msg.quaternion = floor_plane.pose.orientation
                self._normal_publisher.publish(qs_msg)
                ps_msg = PoseStamped()
                ps_msg.header = floor_plane.header
                ps_msg.pose = floor_plane.pose
                self._pose_publisher.publish(ps_msg)
            else:
                rospy.logdebug("[cluster_analysis::FloorFilter.perform] No floor plane found")

    def transform(self, frame_from, frame_to, pose):
        """
        Transform a given pose into another frame using the transform library (tf)
        :param frame_from: frame in which the orientation is valid
        :type frame_from: str
        :param frame_to:target frame for the transformation
        :type frame_to: str
        :param pose: pose that should be transformed
        :type pose: geometry_msgs.msg.Pose
        :return: transformed pose
        :rtype: geometry_msgs.msg.PoseStamped
        """
        # see:  http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
        #       http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20(Python)
        from_exists = self._tf.frameExists(frame_from)
        to_exists = self._tf.frameExists(frame_to)
        if from_exists and to_exists:
            ps = PoseStamped()
            ps.header.frame_id = frame_from
            ps.pose = pose
            self._tf.waitForTransform(frame_from, frame_to, rospy.Time(), rospy.Duration(4))
            try:
                ret_ps = self._tf.transformPose(frame_to, ps)
                return ret_ps
            except Exception as ex:
                rospy.logwarn("[cluster_analysis::FloorFilter.transform] Couldn't Transform from " + frame_from +
                              " to " + frame_to)
                rospy.logwarn(ex.message)
        rospy.logwarn(
            "[cluster_analysis::FloorFilter.transform] Couldn't Transform from " + frame_from + " to " + frame_to)
        return None

    def identify_floor(self, tables):
        """
        Identify the floor plane from a given list
        :param tables: list of planes
        :type tables: object_recognition_msgs.msgs.TableArray
        :return: floor plane
        :rtype: object_recognition_msgs.msgs.Table
        """
        if tables is None:
            rospy.loginfo("[cluster_analysis::FloorFilter.identify_floor] tables is None")
            return None
        ret_plane = None  # type: object_recognition_msgs.msgs.Table
        lst_ret_planes = []
        # Determine planes with a parallel normal vector
        # Determine closest plane to the floor (z=0 @ base_link, floor is approx. 53 cm below base_link)
        min_z = float("inf")
        for plane in tables.tables:
            pose = self.transform(plane.header.frame_id, self.floor_frame, plane.pose)
            if pose is None:
                continue
            # rospy.loginfo("[cluster_analysis::FloorFilter.identify_floor] plane-frame_id = %s, z=%g" % (plane.header.frame_id, plane.pose.position.z))
            # rospy.loginfo("[cluster_analysis::FloorFilter.identify_floor] trans-frame_id = %s, z=%g" % (pose.header.frame_id, pose.pose.position.z))
            # rospy.loginfo("[cluster_analysis::FloorFilter.identify_floor] floor-frame_id = " + self.floor_frame)
            pose = pose.pose
            # rospy.loginfo("[cluster_analysis::FloorFilter.identify_floor]  its quaternion = (" +
            #                str(pose.orientation.x) + "," + str(pose.orientation.y) + "," +
            #                str(pose.orientation.z) + "," + str(pose.orientation.w) + ")")
            # TODO: Check criteria
            z = pose.position.z
            if z < 0 and abs(pose.orientation.z) > (2 * (abs(pose.orientation.x) + abs(pose.orientation.y))):
                v = pose.position.z + self.floor_frame_offset
                rospy.logdebug("[cluster_analysis::FloorFilter.identify_floor] %g = %g + %g" % (
                        v, pose.position.z, self.floor_frame_offset
                ))
                if min_z > v:
                    ret_plane = plane
                    min_z = v
                    rospy.logdebug("[cluster_analysis::FloorFilter.identify_floor] min_z = " + str(min_z))

        if ret_plane is None:
            rospy.logdebug("[cluster_analysis::FloorFilter.identify_floor] ret_plane is None")
        else:
            ret_plane.header = tables.header
        return ret_plane


if __name__ == '__main__':
    rospy.init_node("floor_filter")
    obj = FloorFilter()
    obj.perform()
    rospy.spin()
