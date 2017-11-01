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
import autonomy.CalibrationCameraEEMoveTask
import calibration.CalibrationCameraEEComputation
from geometry_msgs.msg import PoseStamped, Pose
from pyquaternion import Quaternion


def matrix_to_pose(matrix):
    rospy.logdebug("Calibrate_camera_end_effector_transformation.py@matrix_to_pose: matrix.shape = "+str(matrix.shape))
    ret_pose = Pose()
    q = Quaternion(matrix=matrix[0:3, 0:3])
    ret_pose.orientation.w = q.real
    ret_pose.orientation.x = q.imaginary[0]
    ret_pose.orientation.y = q.imaginary[1]
    ret_pose.orientation.z = q.imaginary[2]
    ret_pose.position.x = matrix[0, 3]
    ret_pose.position.y = matrix[1, 3]
    ret_pose.position.z = matrix[2, 3]

    return ret_pose


if __name__ == '__main__':
    rospy.init_node("Calibrate_camera_end_effector_transformation",  log_level=rospy.INFO)

    pub_camera = rospy.Publisher("camera_ee_transformation", PoseStamped, queue_size=1)
    pub_world = rospy.Publisher("base_target_transformation", PoseStamped, queue_size=1)

    camera_frame = rospy.get_param("camera_frame", "gripper_camera_rgb_optical_frame")
    base_frame = rospy.get_param("base_frame", "gripper_ur5_base_link")
    n = len(rospy.get_param("~camera_calib_path"))

    task = autonomy.CalibrationCameraEEMoveTask.CalibrationCameraEEMoveTask()
    computation = calibration.CalibrationCameraEEComputation()

    task.start()
    #  wait till arm reached position
    rospy.sleep(5.)
    computation.get_checkerboard_pose()  # set_Po
    computation.set_transformation()     # set_Pe
    while not rospy.is_shutdown():
        task.start()
        #  wait till arm reached position
        rospy.sleep(5.)
        try:
            computation.get_checkerboard_pose()  # set_Po
            computation.set_transformation()     # set_Pe
            computation.compute()                # calculate Pc, Pw
            if computation.Pc is None or computation.Pw is None:
                rospy.loginfo("Calibrate_camera_end_effector_transformation.py: either Pc or Pw wasn't computed")
                continue
            rospy.loginfo("Calibrate_camera_end_effector_transformation.py: Pc\n"+str(computation.Pc))
            rospy.loginfo("Calibrate_camera_end_effector_transformation.py: det(Pc) = "+str(np.linalg.det(computation.Pc)))
            rospy.loginfo("Calibrate_camera_end_effector_transformation.py: Pw\n"+str(computation.Pw))
            rospy.loginfo("Calibrate_camera_end_effector_transformation.py: det(Pw) = "+str(np.linalg.det(computation.Pw)))
            # Orthogonale Matrizen, deren Determinante eins ist, entsprechen Drehungen. Man spricht dann auch von einer
            # eigentlich orthogonalen Matrix. Orthogonale Matrizen, deren Determinante minus eins ist, stellen
            # Drehspiegelungen dar. Man spricht dann auch von einer uneigentlich orthogonalen Matrix.

        except Exception as ex:
            rospy.logerr("Calibrate_camera_end_effector_transformation.py: Error during computation "
                         "\n" + ex.message)
        try:
            ps = PoseStamped()
            ps.header.stamp = computation.ts
            ps.header.frame_id = camera_frame

            ps.pose = matrix_to_pose(computation.Pc)
            pub_camera.publish(ps)
            ps.header.frame_id = base_frame
            ps.pose = matrix_to_pose(computation.Pw)
            pub_world.publish(ps)
        except Exception as ex:
            rospy.logerr("Calibrate_camera_end_effector_transformation.py: Error during PoseStamped "
                         "transformation:\n"+ex.message)
    rospy.spin()
