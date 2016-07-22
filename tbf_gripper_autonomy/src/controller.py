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
import moveit_commander

import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg

import grasping.haf_client
import grasping.hand
import grasping.arm

class Controller(object):
    # see: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

    def __init__(self):
        self.haf_client = grasping.haf_client.HAFClient()
        self.haf_client.add_grasp_cb_function(self.onGraspSearchCallback)

        self.moveit_controller = grasping.arm.MoveItWrapper()

        self.hand_controller = grasping.hand.HandController()

        self.tf_listener = tf.TransformListener()

    def onGraspSearchCallback(self, grasp_pose, quality=-22):
        self.haf_client.remove_grasp_cb_function(self.onGraspSearchCallback)
        self.hand_controller.openHand()

        # clear octomap at grasp point

        # calculate first pose towards the grasp_pose (TCP traverses a straight line from here)
        approach_vector = rospy.get_param("gripper_approach_vector")
        norm_av = [element/tf.transformations.vector_norm(approach_vector) for element in approach_vector]
        offset = 0.5
        first_pose = geometry_msgs.msg.Pose(orientation=grasp_pose.pose.orientation)
        first_pose.position.x = grasp_pose.pose.x - offset * norm_av[0]
        first_pose.position.y = grasp_pose.pose.y - offset * norm_av[1]
        first_pose.position.y = grasp_pose.pose.z - offset * norm_av[2]
        # from urdf file:
        # <joint name="${name}_tool_connection" type="fixed">
        #     <parent link="${name}_ur5_ee_link"/>
        #     <child link="${name}_robotiq_palm"/>
        #     <origin xyz="0.0535 0 0" rpy="0 -0.75049157835 -1.57"/>
        # </joint>
        grasp_detection_frame = rospy.get_param("grasp_detection_frame", "gripper_camera_rgb_frame")
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(grasp_detection_frame, "/gripper_ur5_ee_link", now, rospy.Duration(4))
        (trans, rot) = self.tf_listener.lookupTransform(grasp_detection_frame, "/gripper_ur5_ee_link", now)
        first_pose = trans*first_pose
        first_pose = rot*first_pose

        # plan path towards the object
        self.group.set_pose_target(first_pose)
        plan1 = self.group.plan()

        # may wait for approval
        rospy.sleep(5)

        # execute plan
        execute_plan = False
        if execute_plan:
            self.group.go(wait=True)
            self.group.clear_pose_targets()

        # grasp object
        self.hand_controller.closeHand()

        # lift grasped object


if __name__ == '__main__':
    rospy.init_node("tubaf_grasping_controller", anonymous=False)
    cntrl = Controller()
    rospy.spin()
