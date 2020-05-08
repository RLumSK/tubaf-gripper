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
import rospy
import tf
import numpy as np
import tubaf_tools.help as tbf

from autonomy.MoveitInterface import MoveitInterface
from tbf_gripper_tools.SmartEquipment import SmartEquipment

from message_filters import Subscriber, Cache
from std_msgs.msg import Header
from object_recognition_msgs.msg import TableArray
from visualization_msgs.msg import MarkerArray
from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest
from grasping.hand import AdvancedHandController
from geometry_msgs.msg import PoseStamped


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def signal_handler(_signal, _frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


if __name__ == '__main__':
    rospy.init_node("test_release", log_level=rospy.DEBUG)

    start_joint_values = [-16.91, -15.0, 110.09, -93.54, 43.17, -135.99]
    home_joint_values = [-180, -90, 0.0, -90, 0.00, 0.0]
    hand = AdvancedHandController()

    tf_listener = tf.TransformListener(rospy.Duration.from_sec(15.0))
    # Init Moveit
    moveit = MoveitInterface("~moveit", tf_listener)  # type: MoveitInterface
    rospy.loginfo("Moveit initialized")
    moveit.move_to_target(start_joint_values, "Start")
    hand.openHand()

    # Equipment Parameter
    equipment_pose_xyz = [-0.99, -0.05, 0.06]
    equipment_pose_rpy = [0.01, -0.01,  -1.44]
    equipment_pose = PoseStamped()
    equipment_pose.header.frame_id = "base_footprint"
    equipment_pose.pose = tbf.array_to_pose(tbf.array_from_xyzrpy(equipment_pose_xyz, equipment_pose_rpy))
    lst_equipment = SmartEquipment.from_parameter_server(group_name="~smart_equipment")
    selected_equipment = lst_equipment[0]  # type: SmartEquipment
    moveit.add_equipment(selected_equipment, pose=equipment_pose)
    rospy.loginfo("Equipment initialized")

    # moving eef
    hand.closeHand(mode="scissor")
    relative_pose = PoseStamped()
    relative_pose.header.frame_id = moveit.eef_link
    relative_pose.pose = tbf.array_to_pose(tbf.array_from_xyzrpy([-0.1, 0, 0], [0, 0, 0]))
    while not moveit.move_to_target(relative_pose, "relative"):
        rospy.sleep(1.0)
    hand.closeHand()

    moveit.move_to_target(home_joint_values, "Home")

