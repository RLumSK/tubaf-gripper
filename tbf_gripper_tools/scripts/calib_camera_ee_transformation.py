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
import tf
import numpy as np
from pyquaternion import Quaternion

if __name__ == '__main__':
    rospy.init_node("calib_camera_ee_transformation")
    base_frame  = rospy.get_param("base_frame", "gripper_ur5_base_link")
    ee_frame  = rospy.get_param("ee_frame", "gripper_ur5_ee_link")
    camera_frame  = rospy.get_param("camera_frame", "gripper_camera_rgb_optical_frame")
    listener = tf.TransformListener("camera_calib_tf_listener")

    Pe_trans = []
    Pe_rot = []
    while not rospy.is_shutdown():
        # Get arm base to end effector transformation
        try:
            (Pe_trans, Pe_rot) = listener.lookupTransform(ee_frame, base_frame, rospy.Time(0)) # return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
            Pe = np.diag(np.ones(4))
            q = Quaternion(Pe_rot[3], Pe_rot[0], Pe_rot[1], Pe_rot[2])
            R = q.rotation_matrix
            Pe[0:3, 0:3] = R[0:3, 0:3]
            Pe[0:3, 3] = Pe_trans[0:3]
            rospy.loginfo(Pe)
            rospy.sleep(2.0)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue