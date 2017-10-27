#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TU Bergakademie Freiberg
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
from tbf_gripper_tools import DirectLinearTransformation
import tf
from pyquaternion import Quaternion

from geometry_msgs.msg import  PoseStamped


class CalibrationCameraEEComputation(object):
    """
    This Class calibrates the rigid transformation between the end-effector and optical camera frame
    """
    def __init__(self):
        """
        Default constructor
        """
        self.listener = tf.TransformListener("ee_move_task_tf_listener")
        self.base_frame = rospy.get_param("base_frame", "gripper_ur5_base_link")
        self.ee_frame = rospy.get_param("ee_frame", "gripper_ur5_ee_link")
        self.pose_topic = rospy.get_param("marker_pose_topic", "/checkerboard/pose_st")

        self.ts = -1
        self.last_Po = np.matrix(np.zeros((4, 4)))
        self.Po = np.matrix(np.zeros((4, 4)))
        self.last_Pe = np.matrix(np.zeros((4, 4)))
        self.Pe = np.matrix(np.zeros((4, 4)))
        self.Pc = np.matrix(np.zeros((4, 4)))
        self.Pw = np.matrix(np.zeros((4, 4)))
        rospy.sleep(.5)

    def set_transformation(self):
        """
        Sets the transformation from base to end-effector at a given time
        :return: -
        :rtype: -
        """
        self.last_Pe = self.Pe
        # Get arm base to end effector transformation
        try:
            rospy.logdebug("CameraCalibComputation.set_transformation(): self.ts=" + self.ts.__str__())
            rospy.logdebug("CameraCalibComputation.set_transformation(): rospy.Time.now()=" + rospy.Time.now().__str__())
            rospy.logdebug("CameraCalibComputation.set_transformation(): delta=" + str(rospy.Time.now()-self.ts))

            #  terminate called after throwing an instance of
            # 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
            #  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
            self.listener.waitForTransform(self.ee_frame, self.base_frame, self.ts, rospy.Duration(10))

            (Pe_trans, Pe_rot) = self.listener.lookupTransform(self.ee_frame, self.base_frame, self.ts)  # return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
            q = Quaternion(Pe_rot[3], Pe_rot[0], Pe_rot[1], Pe_rot[2])
            self.Pe[0:3, 0:3] = q.rotation_matrix
            self.Pe[0:3, 3] = np.reshape(Pe_trans, (3, 1))
            rospy.logdebug("CameraCalibComputation.set_transformation(): ee transformation\n" + self.Pe.__str__())
        except tf.Exception as ex:
            rospy.logerr("CameraCalibComputation.set_transformation(): TF Exception\n" + str(ex))
        except Exception as ex:
            rospy.logerr("CameraCalibComputation.set_transformation(): Exception\n" + ex.message)

    def get_checkerboard_pose(self):
        """
        Get a single PoseStamped
        :return: -
        :rtype: -
        """
        ps = rospy.wait_for_message(self.pose_topic, PoseStamped)
        self.ts = ps.header.stamp
        q = Quaternion(real=ps.pose.orientation.w, imaginary=(ps.pose.orientation.x, ps.pose.orientation.y,
                                                                ps.pose.orientation.z))
        self.last_Po = self.Po

        self.Po[0:3, 0:3] = q.rotation_matrix
        self.Po[:, 3] = np.matrix((ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, 1)).transpose()
        self.Po = CalibrationCameraEEComputation.invertPose(self.Po)

    def compute(self):
        """
        Invoke computation of Pc and Pw
        :return: -
        :rtype: -
        """
        try:
            dlt = DirectLinearTransformation(self.last_Pe, self.last_Po, self.Pe, self.Po)
            rospy.logdebug("CameraCalibComputation.compute(): Pc=\n"+str(dlt.Pc))
            self.Pc = dlt.Pc
            self.Pw = dlt.Pw
        except np.linalg.LinAlgError as ex:
            rospy.logerr("CameraCalibComputation.compute(): "+ex.message)

    @staticmethod
    def invertPose(A):
        """
        Invert a given pose
        :param A: affine transformation as 4x4 matrix
        :type A: numpy.matrix
        :return: inverse transformation
        :rtype: numpy.matrix
        """
        return np.linalg.inv(A)