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
import numpy
import copy

import grasping.haf_client
import grasping.hand
import grasping.arm

import tf.transformations as tfs

from visualization_msgs.msg import *

class Controller(object):
    # see: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

    def __init__(self):
        self.end_effector_link = rospy.get_param("end_effector_link", "gripper_robotiq_palm_planning")

        self.haf_client = grasping.haf_client.HAFClient()
        rospy.loginfo("controller.py: Controller(): initilized HAF client")

        self.haf_client.add_grasp_cb_function(self.onGraspSearchCallback)

        self.moveit_controller = grasping.arm.MoveItWrapper()
        rospy.loginfo("controller.py: Controller(): initilized arm")

        self.hand_controller = grasping.hand.HandController()
        rospy.loginfo("controller.py: Controller(): initilized hand")

        self.tf_listener = tf.TransformListener()

        # rospy.loginfo("controller.py: Controller(): get transformation from ee to gripper")
        # now = rospy.Time.now()
        # # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): approach_vector = %s", approach_vector)
        # self.tf_listener.waitForTransform(self.gripper_link, self.end_effector_link, now, rospy.Duration(4))

        self.haf_client.register_pc_callback()

        # Debugging & Development tools
        self.marker = Marker()
        self.marker_pub = rospy.Publisher("/cntrl_marker", Marker, queue_size=1)

        self.marker.id = 1
        self.marker.ns = "controller"
        self.marker.action = Marker.ADD
        self.marker.type = Marker.ARROW
        # self.marker.mesh_resource = "package://robotiq_s_model_visualization/meshes/s-model_articulated/visual/full_hand.stl"
        self.marker.scale.x = 0.15
        self.marker.scale.y = 0.06
        self.marker.scale.z = 0.03
        self.marker.color.r = 0.1
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 1.0
        self.marker_frame_id = self.end_effector_link

        rospy.loginfo("controller.py: Controller(): finished initialization")

    def calc_box_pose(self, pose):
        approach_vector = rospy.get_param("gripper_approach_vector")
        norm_av = [element/tfs.vector_norm(approach_vector) for element in approach_vector]
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): norm_av = %s", norm_av)
        offset = 0.15
        pose.pose.position.x += offset * norm_av[0]
        pose.pose.position.y += offset * norm_av[1]
        pose.pose.position.y += offset * norm_av[2]
        return pose

    def convert_grasp_pose(self, grasp_pose):
        ret_pose = copy.deepcopy(grasp_pose)
        ret_pose.header.frame_id = self.end_effector_link
        now = rospy.Time.now()

        self.tf_listener.waitForTransform(grasp_pose.header.frame_id, ret_pose.header.frame_id, now, rospy.Duration(4))
        ret_pose = self.tf_listener.transformPose(ret_pose.header.frame_id, ps=grasp_pose)
        return ret_pose

    def onGraspSearchCallback(self, grasp_pose):
        ## Pausing Recognition
        rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): received grasp_pose")  # : %s", grasp_pose)
        self.haf_client.remove_grasp_cb_function(self.onGraspSearchCallback)
        self.haf_client.unregister_pc_callback()

        ## Calculating Target and Passing Pose
        target_pose = self.convert_grasp_pose(grasp_pose)
        self.marker.header = target_pose.header
        self.marker.pose = target_pose.pose
        self.marker_pub.publish(self.marker)
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): publishing marker: %s", self.marker)
        # rospy.sleep(0.5)

        # set an object at grasp point that can collide with the end_effector
        # box_pose = self.calc_box_pose(target_pose)
        # box_name = "one_box"
        # self.moveit_controller.attach_box(box_name, pose=target_pose, size=(0.4, 0.07, 0.25))
        self.hand_controller.openHand()
        rospy.sleep(1.0)
        origin = self.moveit_controller.get_current_joints()

        # Move to Target
        hover_pose = copy.deepcopy(target_pose)
        hover_pose.pose.position.x -= 0.24  # double finger length
        rospy.loginfo("Controller.onGraspSearchCallback(): to hover_pose")
        self.move_to_pose(hover_pose, origin)
        return
        rospy.sleep(5.0)
        rospy.loginfo("Controller.onGraspSearchCallback(): to target_pose")
        self.move_to_pose(target_pose, origin)

        # grasp object
        rospy.loginfo("Controller.onGraspSearchCallback(): closing hand")
        self.hand_controller.closeHand()
        rospy.sleep(3.0)  # wait till the hand grasp the object

        # lift grasped object
        # plan path towards origin
        self.move_to_origin(origin)

        self.hand_controller.openHand()
        rospy.sleep(5.0)  # sleep so the hand can open

        # self.moveit_controller.remove_attached_object(box_name)
        rospy.loginfo("Controller.onGraspSearchCallback(): END")
        self.haf_client.register_pc_callback()
        self.haf_client.add_grasp_cb_function(self.onGraspSearchCallback)

    def move_to_origin(self, origin):
        # plan path towards origin
        if not self.moveit_controller.plan_to_joints(origin):
            # no plan calculated
            rospy.loginfo("Controller.move_to_origin(): couldn't calculate to origin")
            return
        rospy.loginfo("Controller.move_to_origin(): Execution: Moving towards origin")
        # raw_input("Press any key to continue ...")
        executed = False
        while not rospy.is_shutdown() and not executed:
            executed = self.moveit_controller.move_to_pose()
            if executed:
                rospy.loginfo("Controller.move_to_origin(): Execution: Moved to origin ")
                rospy.sleep(3.0)  # wait while the gripper moves towards the pose
            else:
                rospy.sleep(0.5)
                rospy.loginfo("Controller.move_to_origin(): Execution: Try to move towards origin again")

    def move_to_pose(self, pose, origin):
        ## Planing
        rospy.loginfo("Controller.move_to_pose(): planning")
        if not self.moveit_controller.plan_to_pose(pose):
            rospy.loginfo("Controller.move_to_pose(): couldn't calculate a plan")
            if origin is not None:
                self.move_to_origin(origin)
            self.haf_client.register_pc_callback()
            self.haf_client.add_grasp_cb_function(self.onGraspSearchCallback)
            return
        ## Executing
        executed = False
        rospy.loginfo("Controller.move_to_pose(): Execution: Moving towards pose ")
        while not rospy.is_shutdown() and not executed:
            executed = self.moveit_controller.move_to_pose()
            if executed:
                rospy.loginfo("Controller.move_to_pose(): Execution: Moved to pose ")
                rospy.sleep(3.0)  # wait while the gripper moves towards the pose
            else:
                rospy.sleep(0.5)
                rospy.loginfo("Controller.move_to_pose(): Execution: Try to move grasp pose again")

if __name__ == '__main__':
    rospy.init_node("tubaf_grasping_controller", anonymous=False)
    cntrl = Controller()
    rospy.spin()
