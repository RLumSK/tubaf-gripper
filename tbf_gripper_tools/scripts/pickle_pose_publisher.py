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
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose

import cPickle as pkl


def pose_from_matrix(A):
    """
    :param A: affine transformation
    :type A: np.matrix
    :return: pose
    :rtype: Pose
    """
    p = Pose()
    p.position.x = A[0, 3]
    p.position.y = A[1, 3]
    p.position.z = A[2, 3]
    #  q = [x, y, z, w]
    q = tft.quaternion_from_matrix(A)
    p.orientation.w = q[3]
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    return p

if __name__ == '__main__':
    rospy.init_node("Pickle_Pose_Publisher",  log_level=rospy.INFO)
    pub_po = rospy.Publisher("Po", PoseStamped, queue_size=10)
    pub_pe = rospy.Publisher("Pe", PoseStamped, queue_size=10)

    PICKLE_PATH = "/data/pickle/camera_ee_calib/20180531/1/run_0.pickle"
    data = pkl.load(open(PICKLE_PATH, "rb"))
    lst_Po = data[0]
    lst_Pe = data[1]

    while(True):
        for Pe, Po in zip(lst_Pe, lst_Po):
            pose_e = pose_from_matrix(Pe)
            pose_o = pose_from_matrix(Po)
            ps_e = PoseStamped()
            ps_e.pose = pose_e
            ps_e.header.frame_id = "gripper_ur5_base_link"
            print(ps_e)
            pub_pe.publish(ps_e)
            ps_o = PoseStamped()
            ps_o.pose = pose_o
            ps_o.header.frame_id = "gripper_camera_rgb_optical_frame"
            pub_po.publish(ps_o)
            rospy.sleep(rospy.get_param("~pause", 5.0))
