#!/usr/bin/python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, TU Bergakademie Freiberg
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

import signal
import sys

from autonomy.EquipmentTask import *
from tf import TransformListener


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    rospy.init_node("EquipmentTask", log_level=rospy.DEBUG)
    sTg = np.asarray([[0.04356861, 0.99904376, -0.00365364, 0.04911246],
                      [-0.99838966, 0.04340652, -0.03652338, 0.04894902],
                      [-0.03632986, 0.00523903, 0.99932612, 0.18598833],
                      [0., 0., 0., 1.]])
    rospy.loginfo("sTg \n%s" % sTg)
    pub = rospy.Publisher("debug_pose", PoseStamped, queue_size=10)
    joints = None
    obj = EquipmentTask()
    obj.hand_controller.closeHand()
    tf_listener = TransformListener(rospy.Duration.from_sec(15.0))
    while not rospy.is_shutdown() and joints is None:
        ps = sense(tf_listener)
        rospy.loginfo("sense_pose_stamped:\n%s" % ps)
        ps.pose = array_to_pose(optimize_ssb_z_rotation(
            oTs=pose_to_array(ps.pose), sTg=sTg
        ))
        gp = copy.deepcopy(ps)
        gp.pose = array_to_pose(np.matmul(pose_to_array(ps.pose), sTg))
        marker_at_ps(ps, gripper_pose=gp)
        #marker_at_ps(ps)
        pub.publish(ps)
        #joints = obj.moveit.get_ik(gp, ik_link_name="gripper_ur5_wrist_3_joint")
        joints = obj.moveit.plan(target=gp, info="test")
        if joints is not None:
            rospy.loginfo("joints:\n%s" % joints)
        rospy.sleep(2.0)
