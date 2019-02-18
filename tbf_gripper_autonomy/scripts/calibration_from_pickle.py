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
import rospy
from rospkg import RosPack
import os
import pickle
import numpy as np

from autonomy import CalibrationCalculator as calib

if __name__ == '__main__':
    # ROS Setup
    rospy.init_node('calibration_from_pickle', anonymous=True, log_level=rospy.INFO)
    rp = RosPack()
    packages = rp.list()
    def_dir = rp.get_path('tbf_gripper_autonomy')
    fp = rospy.get_param("pickle_dir", os.path.join(def_dir, "data", "pickle", "20181106_2"))
    rospy.logdebug("[CalibrationFrmoPickle] main(): Opening pickle directory: %s" % fp)
    # Import
    runs = []
    for f in os.listdir(fp):
        with (open(os.path.join(fp, f), "rb")) as openfile:
            runs.append(pickle.load(openfile))
    rospy.logdebug("[CalibrationFrmoPickle] main(): Loaded %d pickles" % len(runs))
    # Calculation
    for run in runs:
        # See: tbf_gripper_tools/scripts/pickle_camera_ee_data.py
        lst_Po = run[0][1]
        lst_Pe = run[1][1]
        lst_A = []
        lst_B = []
        for i in range(len(lst_Po)):
            rospy.loginfo("[CalibrationFrmoPickle] main(): Calculationg pose set %d" % (i+1))
            if i+1 == len(lst_Po):
                # Also compute last transformation from end to start configuration
                i = -1
            pe0 = np.asarray(lst_Pe[i])
            pe1 = np.asarray(lst_Pe[i+1])
            rospy.logdebug("[CalibrationFrmoPickle] main(): Pe0 = %s" % pe0)
            rospy.logdebug("[CalibrationFrmoPickle] main(): Pe1 = %s" % pe1)
            # Invert object pose to get camera pose
            pc0 = np.linalg.inv(np.asarray(lst_Po[i]))
            pc1 = np.linalg.inv(np.asarray(lst_Po[i+1]))
            rospy.logdebug("[CalibrationFrmoPickle] main(): Pc0 = %s" % pc0)
            rospy.logdebug("[CalibrationFrmoPickle] main(): Pc1 = %s" % pc1)
            lst_A.append(calib.CalibrationCalculator.calculate_transformation_from_pose(pc0, pc1))
            lst_B.append(calib.CalibrationCalculator.calculate_transformation_from_pose(pe0, pe1))
            rospy.logdebug("[CalibrationFrmoPickle] main(): A = %s" % lst_A[i])
            rospy.logdebug("[CalibrationFrmoPickle] main(): B = %s" % lst_B[i])
        X = calib.CalibrationCalculator.estimate_X(lst_A, lst_B, lam=1)
        if X is None:
            rospy.loginfo("[CalibrationFrmoPickle] main(): no estimate of X")
        else:
            rospy.loginfo("[CalibrationFrmoPickle] main(): X = %s" % X)
