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

import numpy as np
import rospy
# sudo pip install numpy numpy-quaternion

""" @DirectLinearTransformation
implementation of a DLT for the camera calibration routine
@author: Steve Grehl
"""


class DirectLinearTransformation(object):
    """
    This class holds functions to extract a rigid transformation between a camera mounted on an end-effector of a
    robotic arm. Therefore it performs a direct linear transformation using singular value decomposition (svd).
    The public static functions however are generic and may be used for other purposes.
    """

    def __init__(self, lst_Pe, lst_Po):
        """
        construct the optimal camera transformation problem with a end effector pose and object pose
        :param Pe: end effector pose
        :type Pe: numpy.matrix
        :param Po: object pose
        :type Po: numpy.matrix
        """
        self.At = list()
        self.A = []
        for i in range(0, len(lst_Pe)-1):
            Ai = DirectLinearTransformation.__createA(lst_Pe[i], lst_Po[i])
            if i == 0:
                self.A = Ai
            else:
                self.A = DirectLinearTransformation.normMatrix(np.matrix(np.concatenate((self.A, Ai), 0)))
        self.v = DirectLinearTransformation.solveSVD(self.A)
        if self.v is None:
            self.Pc = None
            self.Pw = None
        else:
            self.Pc = self.extractCameraPose(self.v)
            self.Pw = self.extractWorldPose(self.v)
            Cr, Ct = DirectLinearTransformation.orthogonalR(self.Pc[0:3, 0:3], self.Pc[0:3, 3])
            self.Pc = np.diag(np.ones(4))
            self.Pc[0:3, 0:3] = Cr
            self.Pc[0:3, 3] = Ct
            Wr, Wt = DirectLinearTransformation.orthogonalR(self.Pw[0:3, 0:3], self.Pw[0:3, 3])
            self.Pw = np.diag(np.ones(4))
            self.Pw[0:3, 0:3] = Wr
            self.Pw[0:3, 3] = Wt

    def extractCameraPose(self, v):
        """
        construct a pose as 4x4 matrix based on the result of a SVD of A
        :param v: array of unknown variables
        :type v: numpy.ndarray
        :return: pose as 4x4 matrix
        :rtype: numpy.matrix
        """
        C = np.zeros((4,4))
        C[0, 0] = v[0]
        C[1, 0] = v[1]
        C[2, 0] = v[2]
        C[0, 1] = v[3]
        C[1, 1] = v[4]
        C[2, 1] = v[5]
        C[0, 2] = v[6]
        C[1, 2] = v[7]
        C[2, 2] = v[8]
        C[0, 3] = v[9]
        C[1, 3] = v[10]
        C[2, 3] = v[11]
        return C

    def extractWorldPose(self, v):
        """
        construct a pose as 4x4 matrix based on the result of a SVD of A
        :param v: array of unknown variables
        :type v: numpy.ndarray
        :return: pose as 4x4 matrix
        :rtype: numpy.matrix
        """
        W = np.zeros((4,4))
        W[0, 0] = v[12]
        W[1, 0] = v[13]
        W[2, 0] = v[14]
        W[0, 1] = v[15]
        W[1, 1] = v[16]
        W[2, 1] = v[17]
        W[0, 2] = v[18]
        W[1, 2] = v[19]
        W[2, 2] = v[20]
        W[0, 3] = v[21]
        W[1, 3] = v[22]
        W[2, 3] = v[23]
        return W

    @staticmethod
    def orthogonalR(R, t):
        """
        calculate a orthogonal rotation matrix and scale translation
        :param R: rotation matrix in non orthogonal form
        :type R: numpy.matrix
        :param t: translation in the space of R
        :type t: numpy.array
        :return: orthogonal rotation matrix and the scaled translation
        :rtype: tuple numpy.matrix, numpy.array
        """

        (U, S, V) = np.linalg.svd(R)
        tt = t/S
        return np.dot(U, V.transpose()), tt

    @staticmethod
    def solveSVD(A):
        """
        singular value decomposition of A
        :param A: matrix with linear equations
        :type A: numpy.matrix
        :return: vector that solves the problem with min error
        :rtype: numpy.array
        """
        rank = np.linalg.matrix_rank(A)
        if rank < A.shape[1]:
            rospy.logwarn("DirectLinearTransformation.solveSVD(): rank(A) = " + str(rank) +
                          " < A.shape="+str(A.shape))
            return None
        rospy.loginfo("DirectLinearTransformation.solveSVD(): rank(A) = " + str(rank) +
                          " < A.shape="+str(A.shape))
        (U, S, V) = np.linalg.svd(A)
        min_i = np.argmin(S)
        rospy.loginfo("DirectLinearTransformation.solveSVD(): x = \n" + str(V[:, min_i]))
        rospy.loginfo("DirectLinearTransformation.solveSVD(): x_n = \n" + str(V[:, min_i]/V[-1, min_i]))
        return V[:, min_i]/V[-1, min_i]

    @staticmethod
    def normMatrix(A):
        """
        norms each column of the given matrix
        :param A: matrix to norm
        :type A: numpy.matrix
        :return: normed matrix
        :rtype: numpy.matrix
        """
        for j in range(0, len(A[0, :])-1):
            n = np.linalg.norm(A[:, j])
            for i in range(0, len(A[:, j])-1):
                A[i, j] = A[i, j]/n
        return A

    @staticmethod
    def __createA(E, O):
        """
        creates a problem definition based on the end-effector and object transformation
        :param E: affine end-effector transformation as matrix
        :type E: numpy.matrix
        :param O: affine object transformation as matrix
        :type O: numpy.matrix
        :return: A
        :rtype: numpy.matrix
        """
        row = 13
        col = 25
        A = np.zeros((row, col))
        A[0, 0]  = E[0, 0]*O[0, 0]
        A[1, 0]  = E[1, 0]*O[0, 0]
        A[2, 0]  = E[2, 0]*O[0, 0]
        A[3, 0]  = E[0, 0]*O[0, 1]
        A[4, 0]  = E[1, 0]*O[0, 1]
        A[5, 0]  = E[2, 0]*O[0, 1]
        A[6, 0]  = E[0, 0]*O[0, 2]
        A[7, 0]  = E[1, 0]*O[0, 2]
        A[8, 0]  = E[2, 0]*O[0, 2]
        A[9, 0]  = E[0, 0]*O[0, 3]
        A[10, 0] = E[1, 0]*O[0, 3]
        A[11, 0] = E[2, 0]*O[0, 3]

        A[0, 1]  = E[0, 1]*O[0, 0]
        A[1, 1]  = E[1, 1]*O[0, 0]
        A[2, 1]  = E[2, 1]*O[0, 0]
        A[3, 1]  = E[0, 1]*O[0, 1]
        A[4, 1]  = E[1, 1]*O[0, 1]
        A[5, 1]  = E[2, 1]*O[0, 1]
        A[6, 1]  = E[0, 1]*O[0, 2]
        A[7, 1]  = E[1, 1]*O[0, 2]
        A[8, 1]  = E[2, 1]*O[0, 2]
        A[9, 1]  = E[0, 1]*O[0, 3]
        A[10, 1] = E[1, 1]*O[0, 3]
        A[11, 1] = E[2, 1]*O[0, 3]

        A[0, 2]  = E[0, 2]*O[0, 0]
        A[1, 2]  = E[1, 2]*O[0, 0]
        A[2, 2]  = E[2, 2]*O[0, 0]
        A[3, 2]  = E[0, 2]*O[0, 1]
        A[4, 2]  = E[1, 2]*O[0, 1]
        A[5, 2]  = E[2, 2]*O[0, 1]
        A[6, 2]  = E[0, 2]*O[0, 2]
        A[7, 2]  = E[1, 2]*O[0, 2]
        A[8, 2]  = E[2, 2]*O[0, 2]
        A[9, 2]  = E[0, 2]*O[0, 3]
        A[10, 2] = E[1, 2]*O[0, 3]
        A[11, 2] = E[2, 2]*O[0, 3]


        A[0, 3]  = E[0, 0]*O[1, 0]
        A[1, 3]  = E[1, 0]*O[1, 0]
        A[2, 3]  = E[2, 0]*O[1, 0]
        A[3, 3]  = E[0, 0]*O[1, 1]
        A[4, 3]  = E[1, 0]*O[1, 1]
        A[5, 3]  = E[2, 0]*O[1, 1]
        A[6, 3]  = E[0, 0]*O[1, 2]
        A[7, 3]  = E[1, 0]*O[1, 2]
        A[8, 3]  = E[2, 0]*O[1, 2]
        A[9, 3]  = E[0, 0]*O[1, 3]
        A[10, 3] = E[1, 0]*O[1, 3]
        A[11, 3] = E[2, 0]*O[1, 3]

        A[0, 4]  = E[0, 1]*O[1, 0]
        A[1, 4]  = E[1, 1]*O[1, 0]
        A[2, 4]  = E[2, 1]*O[1, 0]
        A[3, 4]  = E[0, 1]*O[1, 1]
        A[4, 4]  = E[1, 1]*O[1, 1]
        A[5, 4]  = E[2, 1]*O[1, 1]
        A[6, 4]  = E[0, 1]*O[1, 2]
        A[7, 4]  = E[1, 1]*O[1, 2]
        A[8, 4]  = E[2, 1]*O[1, 2]
        A[9, 4]  = E[0, 1]*O[1, 3]
        A[10, 4] = E[1, 1]*O[1, 3]
        A[11, 4] = E[2, 1]*O[1, 3]

        A[0, 5]  = E[0, 2]*O[1, 0]
        A[1, 5]  = E[1, 2]*O[1, 0]
        A[2, 5]  = E[2, 2]*O[1, 0]
        A[3, 5]  = E[0, 2]*O[1, 1]
        A[4, 5]  = E[1, 2]*O[1, 1]
        A[5, 5]  = E[2, 2]*O[1, 1]
        A[6, 5]  = E[0, 2]*O[1, 2]
        A[7, 5]  = E[1, 2]*O[1, 2]
        A[8, 5]  = E[2, 2]*O[1, 2]
        A[9, 5]  = E[0, 2]*O[1, 3]
        A[10, 5] = E[1, 2]*O[1, 3]
        A[11, 5] = E[2, 2]*O[1, 3]


        A[0, 6]  = E[0, 0]*O[2, 0]
        A[1, 6]  = E[1, 0]*O[2, 0]
        A[2, 6]  = E[2, 0]*O[2, 0]
        A[3, 6]  = E[0, 0]*O[2, 1]
        A[4, 6]  = E[1, 0]*O[2, 1]
        A[5, 6]  = E[2, 0]*O[2, 1]
        A[6, 6]  = E[0, 0]*O[2, 2]
        A[7, 6]  = E[1, 0]*O[2, 2]
        A[8, 6]  = E[2, 0]*O[2, 2]
        A[9, 6]  = E[0, 0]*O[2, 3]
        A[10, 6] = E[1, 0]*O[2, 3]
        A[11, 6] = E[2, 0]*O[2, 3]

        A[0, 7]  = E[0, 1]*O[2, 0]
        A[1, 7]  = E[1, 1]*O[2, 0]
        A[2, 7]  = E[2, 1]*O[2, 0]
        A[3, 7]  = E[0, 1]*O[2, 1]
        A[4, 7]  = E[1, 1]*O[2, 1]
        A[5, 7]  = E[2, 1]*O[2, 1]
        A[6, 7]  = E[0, 1]*O[2, 2]
        A[7, 7]  = E[1, 1]*O[2, 2]
        A[8, 7]  = E[2, 1]*O[2, 2]
        A[9, 7]  = E[0, 1]*O[2, 3]
        A[10, 7] = E[1, 1]*O[2, 3]
        A[11, 7] = E[2, 1]*O[2, 3]

        A[0, 8]  = E[0, 2]*O[2, 0]
        A[1, 8]  = E[1, 2]*O[2, 0]
        A[2, 8]  = E[2, 2]*O[2, 0]
        A[3, 8]  = E[0, 2]*O[2, 1]
        A[4, 8]  = E[1, 2]*O[2, 1]
        A[5, 8]  = E[2, 2]*O[2, 1]
        A[6, 8]  = E[0, 2]*O[2, 2]
        A[7, 8]  = E[1, 2]*O[2, 2]
        A[8, 8]  = E[2, 2]*O[2, 2]
        A[9, 8]  = E[0, 2]*O[2, 3]
        A[10, 8] = E[1, 2]*O[2, 3]
        A[11, 8] = E[2, 2]*O[2, 3]


        A[9, 9]  = E[0, 0]
        A[10, 9] = E[1, 0]
        A[11, 9] = E[2, 0]
        A[12, 9] = 1

        A[9, 10]  = E[0, 1]
        A[10, 10] = E[1, 1]
        A[11, 10] = E[2, 1]
        A[12, 10] = 1

        A[9, 11]  = E[0, 2]
        A[10, 11] = E[1, 2]
        A[11, 11] = E[2, 2]
        A[12, 11] = 1

        for i in range(12, 24):
            A[i-12, i] = -1

        A[9, 24]  = E[0, 3]
        A[10, 24] = E[1, 3]
        A[11, 24] = E[2, 3]
        A[12, 24] = 1

        return A

if __name__ == '__main__':
    Pe = np.zeros([4, 4])
    Po = np.diag(np.ones(4))

    Pe[0, 2] = 1
    Pe[1, 1] = 1
    Pe[2, 0] = -1
    Pe[3, 3] = 1
    Pe[0, 3] = 0.91
    Pe[1, 3] = -0.11
    Pe[2, 3] = 0.01
    Pe[3, 3] = 1

    Po[0, 0] = 0.999903
    Po[0, 1] = 0.004703
    Po[0, 2] = -0.013133
    Po[0, 3] = -0.105170

    Po[1, 0] = -0.005585
    Po[1, 1] = 0.997669
    Po[1, 2] = -0.068003
    Po[1, 3] = -0.053813

    Po[2, 0] = 0.012783
    Po[2, 1] = 0.068070
    Po[2, 2] = 0.997599
    Po[2, 3] = 0.435377

    transformation = DirectLinearTransformation(Pe, Po)

    print transformation.Pc
    print transformation.Pw