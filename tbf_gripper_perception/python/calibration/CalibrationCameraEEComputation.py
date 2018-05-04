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
import tf
import shelve, os

from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from tbf_gripper_tools import DirectLinearTransformation


class CalibrationCameraEEComputation(object):
    """
    This Class calibrates the rigid transformation between the end-effector and optical camera frame
    """
    def __init__(self):
        """
        Default constructor
        """
        self.debug_fn = "/home/grehl/debug/calibration_camera_ee_computation.shelve"

        self.listener = tf.TransformListener("ee_move_task_tf_listener")
        self.ros_transformer = tf.TransformerROS()
        self.base_frame = rospy.get_param("base_frame", "gripper_ur5_base_link")
        self.ee_frame = rospy.get_param("ee_frame", "gripper_ur5_ee_link")
        self.pose_topic = rospy.get_param("marker_pose_topic", "/checkerboard/pose_st")
        self.dbg_ee_pose_topic = rospy.get_param("ee_pose_topic", "/end_effector/pose_st")
        self.dbg_publisher = rospy.Publisher(self.dbg_ee_pose_topic, PoseStamped, queue_size=10)

        self.ts = -1

        self.lst_Po = list()
        self.lst_Pe = list()

        self.Pc = np.matrix(np.zeros((4, 4)))
        self.Pw = np.matrix(np.zeros((4, 4)))

        rospy.sleep(.5)

    def set_transformation(self):
        """
        Sets the transformation from base to end-effector at a given time
        :return: -
        :rtype: -
        """
        # Get arm base to end effector transformation
        try:
            rospy.logdebug("CameraCalibComputation.set_transformation(): self.ts=" + self.ts.__str__())
            rospy.logdebug("CameraCalibComputation.set_transformation(): rospy.Time.now()=" + rospy.Time.now().__str__())
            rospy.logdebug("CameraCalibComputation.set_transformation(): delta=" + str(rospy.Time.now()-self.ts))

            #  terminate called after throwing an instance of
            # 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
            #  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
            self.listener.waitForTransform(self.ee_frame, self.base_frame, self.ts, rospy.Duration(10))
            (Pe_trans, Pe_q) = self.listener.lookupTransform(self.ee_frame, self.base_frame, self.ts)  # return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
            # # https://answers.ros.org/question/190465/from-transform-to-pose/
            msg_Pe = PoseStamped()
            msg_Pe.header.frame_id = self.ee_frame
            msg_Pe.header.stamp = self.ts
            msg_Pe.pose.position.x = 0
            msg_Pe.pose.position.y = 0
            msg_Pe.pose.position.z = 0
            msg_Pe.pose.orientation.x = 0
            msg_Pe.pose.orientation.y = 0
            msg_Pe.pose.orientation.z = 0
            msg_Pe.pose.orientation.w = 1

            msg_Pe = self.listener.transformPose(self.base_frame, msg_Pe)
            self.dbg_publisher.publish(msg_Pe)

            q = Quaternion(msg_Pe.pose.orientation.w,
                           msg_Pe.pose.orientation.x,
                           msg_Pe.pose.orientation.y,
                           msg_Pe.pose.orientation.z)
            Pe = np.zeros((4, 4))
            Pe[0:3, 0:3] = q.rotation_matrix
            Pe[0, 3] = msg_Pe.pose.orientation.x
            Pe[1, 3] = msg_Pe.pose.orientation.y
            Pe[2, 3] = msg_Pe.pose.orientation.z
            Pe[3, 3] = 1
            rospy.logdebug("CameraCalibComputation.set_transformation(): ee transformation\n" + Pe.__str__())
            return Pe
        except tf.Exception as ex:
            rospy.logerr("CameraCalibComputation.set_transformation(): TF Exception\n" + str(ex))
        except Exception as ex:
            rospy.logerr("CameraCalibComputation.set_transformation(): Exception\n" + ex.message)

    def get_checkerboard_pose(self):
        """
        Get a single PoseStamped
        :return: Current pose
        :rtype: PoseStamped
        """
        ps = rospy.wait_for_message(self.pose_topic, PoseStamped)
        rospy.logdebug("CameraCalibComputation.get_checkerboard_pose(): Received Pose: %s", ps)
        self.ts = ps.header.stamp
        q = Quaternion(real=ps.pose.orientation.w, imaginary=(ps.pose.orientation.x, ps.pose.orientation.y,
                                                                ps.pose.orientation.z))

        Po = np.zeros((4, 4))
        Po[0:3, 0:3] = q.rotation_matrix
        Po[:, 3] = np.matrix((ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, 1))
        self.lst_Po.append(Po) #  Row major
        return Po
        # self.Po = CalibrationCameraEEComputation.invertPose(self.Po)

    def compute(self):
        """
        Invoke computation of Pc and Pw
        :return: -
        :rtype: -
        """
        try:
            dlt = DirectLinearTransformation(self.lst_Pe, self.lst_Po)
            rospy.logdebug("CameraCalibComputation.compute(): Pc=\n"+str(dlt.Pc))
            self.Pc = dlt.Pc
            self.Pw = dlt.Pw
            # self.print_debug()
        except np.linalg.LinAlgError as ex:
            rospy.logerr("CameraCalibComputation.compute(): "+ex.message)

    def print_debug(self):
        """
        Debug output
        :return: -
        :rtype: -
        """
        L = np.dot(self.lst_Pe[-1], self.Pc)
        T = np.dot(L, self.lst_Po[-1])
        G = T-self.Pw
        rospy.loginfo("CameraCalibComputation.debug():\n"
                      "Pw=" + str(self.Pw) + "\n"
                      "Pe[-1]=" + str(self.lst_Pe[-1]) + "\n"
                      "Pc=" + str(self.Pc) + "\n"
                      "Po[-1]=" + str(self.lst_Po[-1]) + "\n"
                      "----------------------------\n"
                      "L =" + str(L) + "\n"
                      "T =" + str(T) + "\n"
                      "G =" + str(G) + "\n"
                      "----------------------------\n"
                      "----------------------------\n"
                      )
        debug_file = shelve.open(self.debug_fn)
        debug_file["Pw"] = self.Pw
        debug_file["Pe"] = self.lst_Pe[-1]
        debug_file["Pc"] = self.Pc
        debug_file["Po"] = self.lst_Po[-1]
        debug_file.close()

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