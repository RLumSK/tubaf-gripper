#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, TU Bergakademie Freiberg
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
#author_ grehl

import rospy
import numpy as np
import ros_numpy as rp
from geometry_msgs.msg import Pose, Transform, Quaternion, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tubaf_tools.srv import GetCameraTransform, GetCameraTransformRequest, GetCameraTransformResponse
from open3d import *
from message_filters import Cache, Subscriber
from tf import TransformListener


SERVICE_NAME = "calc_point_cloud_transform"


class CloudTransform(object):

    def __init__(self, cloud_topic, threshold=0.02):
        self.threshold = threshold
        self.tf_listener = TransformListener()
        sub = Subscriber(cloud_topic, PointCloud2)
        self.cloud_cache = Cache(sub, 200)  # should be a rough minute
        self.old_cloud = None
        self.old_transform = None
        self.base_frame = rospy.get_param("~base_frame", "gripper_ur5_base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "gripper_camera_rgb_frame")
        self.service_name = rospy.get_param("~service_name", SERVICE_NAME)
        self.service = None
        try:
            rospy.logdebug("Sleep ...")
            rospy.sleep(2.0)
            rospy.logdebug("Wake up!")
            self.tf_listener.waitForTransform(self.camera_frame, self.base_frame, rospy.Time(0), rospy.Duration(4.0))
        except Exception, e:
            rospy.logerr("[compute_cloud_transform.py] CloudTransform.__init__: %s" % e)
            raise e

    def init_service(self):
        """
        Initialize the service and variables
        :return: -
        :rtype: -
        """
        try:
            while self.cloud_cache.getLast() is None and not rospy.is_shutdown():
                rospy.sleep(0.2)
            self.old_cloud = self.cloud_cache.getLast()  # type: PointCloud2
            time = self.old_cloud.header.stamp
            self.tf_listener.waitForTransform(self.camera_frame, self.base_frame,  time, rospy.Duration(4))
            (position, orientation) = self.tf_listener.lookupTransform(self.camera_frame, self.base_frame, time)
            self.old_transform = Pose(position=Point(*position), orientation=Quaternion(*orientation))
            self.service = rospy.Service(self.service_name, GetCameraTransform, self.service_call)
        except Exception, e:
            rospy.logwarn("[compute_cloud_transform.py] CloudTransform.init_service: %s" % e)
            self.init_service()

    def service_call(self, req):
        """
        GetCameraTransform service call
        :param req: service request with a timestamp
        :type req: GetCameraTransformRequest
        :return: service response with calculated transformation
        :rtype: GetCameraTransformResponse
        """
        res = GetCameraTransformResponse()
        time = req.time_stamp
        self.tf_listener.waitForTransform(self.camera_frame, self.base_frame, time, rospy.Duration(4))
        (position, orientation) = self.tf_listener.lookupTransform(self.camera_frame, self.base_frame, time)
        new_transform = Pose(position=Point(*position), orientation=Quaternion(*orientation))
        init_transform = CloudTransform.get_relative_transform(new_transform, self.old_transform)
        new_cloud = self.cloud_cache.getElemBeforeTime(time)
        res.transformation = self.calc_transform(self.old_cloud, new_cloud, init_transform)
        self.old_cloud = new_cloud
        return res

    @staticmethod
    def get_relative_transform(new_pose, old_pose):
        """
        Given two Pose from the same reference frame, calculate the relative transformation as pose
        :param new_pose: actual pose
        :type new_pose: Pose
        :param old_pose: former pose
        :type old_pose: Pose
        :return: relative transformation as pose
        :rtype: Pose
        """
        p1 = rp.numpify(new_pose)
        p0 = rp.numpify(old_pose)
        t = np.matmul(np.linalg.inv(p0), p1)
        return rp.msgify(Pose, t)

    def calc_transform(self, old_cloud, new_cloud, init_transform):
        """
        Calculate the transformation between to pointclouds using the given transformation
        :param old_cloud: first point cloud
        :type old_cloud: PointCloud2
        :param new_cloud: second point cloud
        :type new_cloud: PointCloud2
        :param init_transform: initial transformation
        :type init_transform: Pose
        :return: calculated transformation
        :rtype: Transform
        """
        cloud0 = open3d.geometry.PointCloud()
        for point in pc2.read_points(old_cloud, skip_nans=True, field_names=("x", "y", "z")):
            cloud0.points.append(np.asarray([point[0], point[1], point[2]]))

        cloud1 = open3d.geometry.PointCloud()
        for point in pc2.read_points(new_cloud, skip_nans=True, field_names=("x", "y", "z")):
            cloud1.points.append(np.asarray([point[0], point[1], point[2]]))

        init = rp.numpify(init_transform)

        rospy.logdebug("[CloudTransform.calc_transform()] init: %s" % init)

        reg_p2p = registration_icp(cloud0, cloud1, self.threshold, init, TransformationEstimationPointToPoint())

        rospy.logdebug("[CloudTransform.calc_transform()] reg_p2p: %s\n%s\ntype:%s" % (reg_p2p, reg_p2p.transformation,
                                                                                       type(reg_p2p.transformation)))

        pose = rp.msgify(msg_type=Pose, numpy_obj=reg_p2p.transformation)  # type: Pose
        res = Transform()
        res.translation = pose.position
        res.rotation = pose.orientation
        return res


def test_service():
    """
    Test the Service
    :return:
    :rtype:
    """
    ## Testing ###
    rospy.wait_for_service(SERVICE_NAME)

    req = GetCameraTransformRequest()
    req.time_stamp = rospy.Time.now()
    res = None
    try:
        calc_point_cloud_transform = rospy.ServiceProxy(SERVICE_NAME, GetCameraTransform)
        res = calc_point_cloud_transform(req)
    except rospy.ServiceException, e:
        rospy.logwarn("[compute_cloud_transform.py@test_service] Service call failed: %s" % e)

    rospy.loginfo("[compute_cloud_transform.py@main] Response: %s" % res)


if __name__ == '__main__':
    rospy.init_node('compute_cloud_transform', log_level=rospy.WARN)
    threshold = rospy.get_param("~threshold", 0.02)
    cloud_topic = rospy.get_param("~cloud_topic", "/gripper_camera/depth/points")
    ct = CloudTransform(cloud_topic, threshold=threshold)
    ct.init_service()
    rospy.loginfo("[compute_cloud_transform] %s is ready" % SERVICE_NAME)

    if rospy.get_param("test", True):
        test_service()
    rospy.spin()
