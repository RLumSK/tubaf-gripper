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
# author: grehl

import rospy
import importlib

from geometry_msgs.msg import Transform


class CameraServiceInterface(object):
    """
    Action Server in ROS for a camera
    """

    def __init__(self, c_name):
        """
        Default constructor
        :param c_name: name of the camera
        :type c_name: str
        """
        self.camera_name = c_name
        # rospy.init_node(self.camera_name+"_service_interface", log_level=rospy.DEBUG)
        self.service_get_camera_transform = None
        self.service_type_str = rospy.get_param("~odom_service_type", "GetOdometryAt")
        service_package = rospy.get_param("~odom_service_pkg", "rgbd_odometry_service")
        print service_package
        self.srv = importlib.import_module(".srv", service_package)
        self.service_class = eval("self.srv."+self.service_type_str)
        self.request_class = eval("self.srv."+self.service_type_str+"Request")
        self.response_class = eval("self.srv."+self.service_type_str+"Response")
        self.py_translation_str = rospy.get_param("~py_translation_str", ".odometry.pose.pose.position")
        self.py_rotation_str = rospy.get_param("~py_rotation_str", ".odometry.pose.pose.orientation")
        self.py_time_str = rospy.get_param("~py_time_str", ".t0")
        remote_service_name = rospy.get_param("~odom_service_name", "get_odom_at")
        rospy.loginfo("[CameraServiceInterface.__init__()] Waiting for service: %s" % remote_service_name)
        rospy.wait_for_service(remote_service_name)
        self.service = rospy.ServiceProxy(remote_service_name, self.service_class)

    def init_services(self):
        """
        Start ROS-Service
        :return: -
        :rtype: None
        """
        self.service_get_camera_transform = rospy.Service(self.get_service_name("transform"), self.service_class,
                                                          self.get_camera_transform)

    def get_camera_transform(self, ts):
        """
        Implementation of the service call for the camera transform
        :param ts: time_stamp
        :type ts: Time
        :return: camera transformation
        :rtype: Transform
        """
        if self.service_get_camera_transform is None:
            self.init_services()
        odom_req = self.request_class()
        exec('odom_req'+self.py_time_str+'=ts')
        odom_res = self.service(odom_req)

        res = Transform()
        res.translation = eval("odom_res"+self.py_translation_str)
        res.rotation = eval("odom_res"+self.py_rotation_str)

        return res

    def get_service_name(self, serivce_type):
        """
        Get the service name for the given type: Syntax is 'get_'+camera_name+'_'+type,
        e.g. get_gripper_camera_transform
        :param serivce_type: service type
        :type serivce_type: str
        :return: service name
        :rtype: str
        """
        return "get_"+self.camera_name+"_"+serivce_type

    def test_service(self):
        """
        Test the service by implementing a client
        :return: -
        :rtype: -
        """
        while not rospy.is_shutdown():
            try:
                res = self.get_camera_transform(rospy.Time.now())
                rospy.loginfo("[camera_service_monitor.test_service()] response: %s" % res)
                rospy.sleep(2.0)
            except rospy.ServiceException, e:
                rospy.logerr("[camera_service_monitor.test_service()] Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node('camera_service_interface', log_level=rospy.INFO)
    rospy.loginfo("[camera_service_interface] Start")
    name = rospy.get_param("~name", "gripper_camera")
    camera_action_server = CameraServiceInterface(name)
    rospy.loginfo("[camera_service_interface] camera_action_server initialized")
    camera_action_server.init_services()
    rospy.loginfo("[camera_service_interface] services initialized")

    camera_action_server.test_service()

    rospy.spin()
