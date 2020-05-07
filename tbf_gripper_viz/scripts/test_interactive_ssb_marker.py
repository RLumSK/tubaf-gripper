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
from tf import TransformListener
from tbf_gripper_rviz.ssb_marker import SSBMarker
from tbf_gripper_tools.SmartEquipment import SmartEquipment
from autonomy.MoveitInterface import MoveitInterface


if __name__ == "__main__":
    rospy.init_node("SSB_marker", log_level=rospy.DEBUG)
    tf_listener = TransformListener(rospy.Duration.from_sec(15.0))
    mvit = MoveitInterface(tf_listener=tf_listener)  # type: MoveitInterface
    lst_equipment = SmartEquipment.from_parameter_server(group_name="~smart_equipment")
    lst_marker = []  # type: list
    for se in lst_equipment:  # type: SmartEquipment
        se.calculate_grasp_offset(attached_frame="gripper_robotiq_palm_planning", tf_listener=tf_listener)
        rospy.loginfo(se.grasp_offset)
        v_marker = SSBMarker.from_SmartEquipment(se, tf_listener)
        mvit.clear_octomap_on_marker(v_marker)
        lst_marker.append(v_marker)
    while True:
        for marker in lst_marker:  # type: SSBMarker
            marker.enable_marker()
            rospy.sleep(5.0)
            marker.disable_marker()

    rospy.spin()
