#!/usr/bin/python2.7
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
import autonomy.CalibrationCameraEEMoveTask
import calibration.CalibrationCameraEEComputation
from geometry_msgs.msg import PoseStamped, Pose
from pyquaternion import Quaternion

import cPickle as pkl


def print_info(data):
    """
    Print debug information about the saved data
    :param data: list with Pe Po
    :type data: lst
    :return: -
    :rtype: -
    """
    rospy.loginfo("print_info(): len(data) %d" % len(data))
    rospy.loginfo("print_info(): len(data[0]) %d" % len(data[0]))
    rospy.loginfo("print_info(): len(data[1]) %d" % len(data[1]))
    rospy.loginfo("print_info(): len(data[0][0]) %d" % len(data[0][0]))
    rospy.loginfo("print_info(): len(data[1][0]) %d" % len(data[1][0]))

    rospy.loginfo("print_info(): data[0][0])\n %s" % str(data[0][0]))
    rospy.loginfo("print_info(): data[1][0])\n %s" % str(data[1][0]))


def matrix_to_pose(matrix):
    rospy.logdebug("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: matrix.shape = "+str(matrix.shape))
    ret_pose = Pose()
    if np.linalg.det(matrix[0:3, 0:3]) == -1.0:
        #  http://www.mathe-lerntipps.de/geometrie/symmetrieverhalten.html
        #  -1 = Drehspieglung -> bedeutet Rotation, dann Inversion
        #  Da die Matrix orthogonal ist gilt: Inverse = Transponierte
        #  --> Umkehrung der Invertierung
        matrix[0:3, 0:3] = matrix[0:3, 0:3].transpose()

    rospy.loginfo("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: TEST ")
    try:
        q = Quaternion(matrix=matrix[0:3, 0:3])
        rospy.loginfo("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: TEST ")
        ret_pose.orientation.w = q.real
        ret_pose.orientation.x = q.imaginary[0]
        ret_pose.orientation.y = q.imaginary[1]
        ret_pose.orientation.z = q.imaginary[2]
        rospy.loginfo("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: TEST ")
        ret_pose.position.x = matrix[0, 3]
        ret_pose.position.y = matrix[1, 3]
        ret_pose.position.z = matrix[2, 3]
    except Exception as ex:
        rospy.logerr("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: Error during pose convertion " +
                     str(ex.__class__) +
                     "\n" + ex.message)
        rospy.loginfo("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: det() = "+str(np.linalg.det(matrix[0:3, 0:3])))
        return None
    return ret_pose


if __name__ == '__main__':
    rospy.init_node("Pickle_Camera_EE_Data",  log_level=rospy.INFO)

    pub_camera = rospy.Publisher("camera_ee_transformation", PoseStamped, queue_size=1)
    pub_world = rospy.Publisher("base_target_transformation", PoseStamped, queue_size=1)

    camera_frame = rospy.get_param("~camera_frame", "gripper_camera_rgb_optical_frame")
    base_frame = rospy.get_param("~base_frame", "gripper_ur5_base_link")
    pickle_fn = rospy.get_param("~pickle_fn", "camera_ee_data.pickle")
    pause = rospy.get_param("~pause", 5.0)

    for n_run in range(0, 10):
        task = autonomy.CalibrationCameraEEMoveTask.CalibrationCameraEEMoveTask()
        computation = calibration.CalibrationCameraEEComputation()
        rospy.logdebug("pickle_camera_ee_data.py: Starting task ... ")
        while not rospy.is_shutdown() and not task.is_redundant:
            i = task.index
            task.start()
            while i == task.index:
                rospy.sleep(0.3)
                #  wait till arm reached position
            rospy.sleep(pause)
            try:
                Po = computation.get_checkerboard_pose()  # set_Po
                qo = Quaternion(matrix=Po)
                Pe = computation.set_transformation()     # set_Pe
                qe = Quaternion(matrix=Pe)
                rospy.loginfo("pickle_camera_ee_data.py@main():\nPo:\n %s \nq(Po)= %s \nPe:\n %s \nq(Pe)= %s" %
                              (Po, qo, Pe, qe))
            except Exception as ex:
                rospy.logerr("pickle_camera_ee_data.py: Error during acquisition "
                             "\n" + ex.message)

        # Pickle it
        data = [['Po', computation.lst_Po], ['Pe', computation.lst_Pe]]
        print_info(data)
        rospy.loginfo("pickle_camera_ee_data.py: Starting pickle ... ")
        pkl.dump(data, open(pickle_fn+"_"+str(n_run)+".pickle", "wb"))
        rospy.loginfo("pickle_camera_ee_data.py: Finished pickle ")

    # rospy.spin()
