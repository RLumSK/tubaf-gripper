#!/usr/bin/python
# Software License Agreement (MIT License)
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

# Related work:
# [1] ANDREFF, Nicolas; HORAUD, Radu; ESPIAU, Bernard.
# Robot hand-eye calibration using structure-from-motion.
# The International Journal of Robotics Research, 2001, 20. Jg., Nr. 3, S. 228-248.
import rospy
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import lsq_linear
import geomstats as gs
import geomstats.rigid_transformations as transform

class CalibrationCalculator:
    """
    Core Calculation class for the hand-eye-calibration routine
    It basically implemts the Paper "Robot Hand-Eye Calibration Using Structure-from-Motion" from Nicolas Andreff,
    Radu Horaud and Bernard Espiau
    """

    # def __init__(self):
    #     """
    #     Default constructor
    #     """

    @staticmethod
    def estimate_X(lst_A, lst_B, lam=1):
        """
        Estimate the transformation from camera (eye) to gripper (hand)
        :param lst_A: list with affine transformations of the camera
        :param lst_B: list with affine transformations of the gripper
        :param lam: scale factor
        :return: one transformation (camera to gripper)
        """
        if len(lst_A) != len(lst_B):
            rospy.logerr("CalibrationCalculator.estimate_X(): len(lst_A) = %d != %d len(lst_B)\n Number of given "
                         "transformations for A, B not equal. Returning None"% (len(lst_A), len(lst_B)))
            return None
        if len(lst_A) <= 2:
            rospy.logerr("CalibrationCalculator.estimate_X(): len(lst_A) = %d <= 2\n Number of given "
                         "transformations for A and B to low. Returning None"% (len(lst_A)))
            return None
        #Equation (41)
        n = len(lst_A)
        mat_41 = np.zeros([9*n, 9])
        for pose_A, pose_B, i in (lst_A, lst_B, range(0, n)):
            mat_41[9*i:9*2*i, :] = np.identity(9) - np.kron(pose_A[:3, :3], pose_B[:3, :3])
        Rx = np.reshape(null_space(mat_41), [3, 3])

        #Equation (42)
        mat_42_l = np.zeros([3*n, 4])
        vec_42_r = np.zeros([3*n])
        for pose_A, pose_B, i in (lst_A, lst_B, range(0, n)):
            # From Equation (6)
            u = pose_A[3,:2]/lam
            mat_42_l[3*i:3*2*i, :] = [np.identity(3)-pose_A[:3, :3], -u]
            vec_42_r[3*i:3*2*i] = np.dot(-Rx, pose_B[3, :3])
        vec_42_l = lsq_linear(mat_42_l, vec_42_r)
        tx = vec_42_l[:3]
        lam_calc = vec_42_l[3]
        X = np.identity(4)
        X[:3, :3] = Rx
        X[3, :3] = tx
        return X

    @staticmethod
    def calculate_transformation_from_pose(p0, p1):
        """
        Calculate the affine transformation between two given pose
        :param p0: starting pose as affine transformation
        :param p1: ending pose as affine transformation
        :return: affine transformation between p0 and p1
        """
        import numpy as np
        return np.matmul(p1, np.linalg.inv(p0))

    @staticmethod
    def average_X(lst_X):
        """
        Averaging all given affine transformations
        :param lst_X: a list of affine transformations
        :return: affine transformation
        """
        raise NotImplementedError()
        # transform.
        # lst_R = []
        # lst_t = []
        # for X in lst_X:
        #     lst_R.append(gs.rotation_vector_from_rotation_matrix(X[:3, :3]))
        #     lst_t.append(X[3, :3])
        #
        # # Mean translation
        # t = np.nanmean(lst_t)
        # std_t = np.nanstd(lst_t)
        #
        # # Mean rotation
        #
        # gs.rotation_vector_from_rotation_matrix(R)


