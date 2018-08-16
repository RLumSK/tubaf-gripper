#!/usr/bin/python
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, TU Bergakademie Freiberg
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

import cPickle as pkl
import numpy as np
from ruamel.yaml import YAML


def import_from_pickle(path):
    """
    Import the data from a pickle file
    :param path: Path to the file
    :type path: basestring
    :return: set of A and B, A describes the gripper motion, B describes the camera motion
    :rtype: set
    """
    data = pkl.load(open(path, "rb"))
    Pe = data[0]  # gripper pose
    Po = data[1]  # camera pose
    A, B = [], []
    lst_A, lst_B = [], []
    for i in range(0, len(Pe)-2, 2):
        # E0 = 0A1 * E1 --> 0A1 = E0 * E1^-1
        E0 = np.matrix(Pe[i])
        E1 = np.matrix(Pe[i+1])
        A = np.dot(E1, np.linalg.inv(E0))
        print('E0', E0)
        print('E1', E1)
        print('A', A)
        O0 = np.matrix(Po[i])
        O1 = np.matrix(Po[i+1])
        B = np.matmul(O1, np.linalg.inv(O0))
        lst_A.append(A)
        lst_B.append(B)
    return A, B, lst_A, lst_B


def rot(T):
    """
    Extract rotation matrix from an affine transformation
    :param T: affine transformation
    :type T: numpy.matrix
    :return: rotation matrix
    :rtype: numpy.matrix
    """
    return T[:3, :3]


def trans(T):
    """
    Extract translation vector from an affine transformation
    :param T: affine transformation
    :type T: numpy.matrix
    :return: translation
    :rtype: numpy.array
    """
    return T[:3, 3]


def vec(M):
    """
    Linear operator vec as stated in [1] section 2.1. Notation
    :param M: matrix
    :type M: numpy.matrix
    :return: vector
    :rtype: numpy.array
    """
    return M.flatten()


def product(M, N):
    """
    Kronecker product as stated in [1] section 2.1. Notation
    :param M: left matrix
    :type M: numpy.matrix
    :param N: right matrix
    :type N: numpy.matrix
    :return: matrix
    :rtype: numpy.matrix
    """
    return np.kron(M, N)


def generate_hls(A, B):
    """
    Consult [1] equation (14) - This method generates the first matrix
    :param A: transformation from camera[0] to camera[1]
    :type A: numpy.matrix
    :param B: transformation from gripper[0] to gripper[1]
    :type B: numpy.matrix
    :return: coefficient matrix
    :rtype: numpy.matrix
    """
    ret_matrix = np.empty([12, 13])
    ret_matrix[:9, :9] = np.identity(9) - product(rot(A), rot(B))
    ret_matrix[9:12, :9] = product(np.matrix(np.identity(3)), trans(B).T)
    ret_matrix[9:12, 9:12] = np.identity(3) - rot(A)
    ret_matrix[9:12, 12] = -trans(A).T
    return ret_matrix


def null(G):
    """
    Solve linear system using numpy
    :param G: coefficient matrix
    :type G: numpy.matrix
    :return: solution vector
    :rtype: numpy.array
    """
    return np.linalg.lstsq(G, np.zeros([12, 1]))


def export_solution(A, B, x, path):
    """
    Write the parameter and solution in a file
    :param A: camera transformation
    :type A: numpy.matrix
    :param B: gripper transformation
    :type B: numpy.matrix
    :param x: solution vector
    :type x: numpy.ndarray
    :param path: path to to the file
    :type path: basestring
    :return: -
    :rtype: -
    """
    X = np.identity(4)
    X[:3, :3] = x[:9].reshape([3, 3])
    X[:3, 3] = x[9:12].T

    fp = open(path, "w")
    yaml =YAML()
    yaml.dump({'A': A.tolist()}, fp)
    yaml.dump({'B': B.tolist()}, fp)
    yaml.dump({'X': X.tolist()}, fp)
    yaml.dump({"lambda": x[12].tolist()}, fp)
    fp.close()


def lst_export_solution(lstA, lstB, lstx, path):
    """
    Write the parameter and solution in a file
    """
    fp = open(path, "w")
    yaml =YAML()

    for i in range(len(lstA)):
        X = np.identity(4)
        x = lstx[i]
        X[:3, :3] = x[:9].reshape([3, 3])
        X[:3, 3] = x[9:12].T
        yaml.dump({'A': lstA[i].tolist()}, fp)
        yaml.dump({'B': lstB[i].tolist()}, fp)
        yaml.dump({'X': X.tolist()}, fp)
        yaml.dump({"lambda": x[12].tolist()}, fp)
    fp.close()


if __name__ == '__main__':
    for run_id in range(0,9,1):
        # Import data from Pickle
        path = "/data/pickle/camera_ee_calib/20180531/0/run_"+str(run_id)+".pickle"
        (A, B, lst_A, lst_B) = import_from_pickle(path)
        # Generate homogeneous linear system (14)
        G = generate_hls(A, B)
        lst_x = []
        for i in range(len(lst_A)):
            G = generate_hls(lst_A[i], lst_B[i])
            (x, res, ran, s) = null(G)
            print("Rang: %s, #Variablen: %s" % (ran, G.shape[0]))
            if ran < G.shape[0]:
                print("unendlich viele Loesungen")
            elif ran == G.shape[0]:
                print("nur triviale Loesung")
            else:
                print("Rang der Koeffizientanmatrix ist groesser als die Zahl der Variablen")
            lst_x.append(x)
        # Solve homogeneous linear system
        #(x, residuals, rank, s) = null(G)
        # Save solution
        export_path = "/data/pickle/camera_ee_calib/20180531/0/run_"+str(run_id)+".solution"
        #export_solution(A, B, x, export_path)
        lst_export_solution(lst_A, lst_B, lst_x, export_path+"_lst")
