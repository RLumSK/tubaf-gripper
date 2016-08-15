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

import grasping.haf_client
import grasping.hand
import grasping.arm

import tf.transformations as tfs

from visualization_msgs.msg import *

class Controller(object):
    # see: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

    def __init__(self):
        self.end_effector_link = rospy.get_param("end_effector_link", "/gripper_ur5_ee_link")

        self.haf_client = grasping.haf_client.HAFClient()
        rospy.loginfo("controller.py: Controller(): initilized HAF client")

        self.haf_client.add_grasp_cb_function(self.onGraspSearchCallback)

        self.moveit_controller = grasping.arm.MoveItWrapper()
        rospy.loginfo("controller.py: Controller(): initilized arm")

        self.hand_controller = grasping.hand.HandController()
        rospy.loginfo("controller.py: Controller(): initilized hand")

        self.tf_listener = tf.TransformListener()

        self.haf_client.register_pc_callback()

        # Debugging & Development tools
        self.marker = Marker()
        self.marker_pub = rospy.Publisher("cntrl_marker", Marker, queue_size=1)

        self.marker.id = 1
        self.marker.ns = "controller"
        self.marker.action = Marker.ADD
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = "package://robotiq_s_model_visualization/meshes/s-model_articulated/visual/full_hand.stl"
        self.marker.scale.x = 1.
        self.marker.scale.y = 1.
        self.marker.scale.z = 1.
        self.marker.color.r = 0.1
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 1.0
        self.marker_frame_id = "/gripper_robotiq_palm"

        rospy.loginfo("controller.py: Controller(): finished initialization")

    def onGraspSearchCallback(self, grasp_pose):
        rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): received grasp_pose")  # : %s", grasp_pose)
        # self.haf_client.remove_grasp_cb_function(self.onGraspSearchCallback)
        self.hand_controller.openHand()

        # clear octomap at grasp point

        # calculate first pose towards the grasp_pose (TCP traverses a straight line from here)
        # from urdf file:
        # <joint name="${name}_tool_connection" type="fixed">
        #     <parent link="${name}_ur5_ee_link"/>
        #     <child link="${name}_robotiq_palm"/>
        #     <origin xyz="0.0535 0 0" rpy="0 -0.75049157835 -1.57"/>
        # </joint>
        grasp_detection_frame = grasp_pose.header.frame_id
        approach_vector = rospy.get_param("gripper_approach_vector")
        now = rospy.Time.now()
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): approach_vector = %s", approach_vector)
        self.tf_listener.waitForTransform(grasp_detection_frame, self.end_effector_link, now, rospy.Duration(4))
        norm_av = [element/tfs.vector_norm(approach_vector) for element in approach_vector]
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): norm_av = %s", norm_av)
        offset = 0.25
        grasp_pose.pose.position.x += offset * norm_av[0]
        grasp_pose.pose.position.y += offset * norm_av[1]
        grasp_pose.pose.position.y += offset * norm_av[2]
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): grasp_pose = %s", grasp_pose)
        first_pose = self.tf_listener.transformPose(self.end_effector_link, grasp_pose)
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): first_pose = %s", first_pose)

        # DEBUG
        self.marker.header = first_pose.header
        self.marker.pose = first_pose.pose
        rospy.loginfo("Controller.onGraspSearchCallback(): publishing marker")
        self.marker_pub.publish(self.marker)
        # DEBUG END

        # plan path towards the object
        if not self.moveit_controller.plan_to_pose(first_pose):
            # no plan calculated
            rospy.loginfo("Controller.onGraspSearchCallback(): couldn't calculate a plan, trying with next pose")
            return
        rospy.loginfo("Controller.onGraspSearchCallback(): sleeping while waiting for approval")
        # may wait for approval
        rospy.sleep(5)

        # execute plan
        execute_plan = True
        if execute_plan:
            rospy.loginfo("Controller.onGraspSearchCallback(): Execution")
            self.moveit_controller.move_to_pose()

        # grasp object
        rospy.loginfo("Controller.onGraspSearchCallback(): closing hand")
        self.hand_controller.closeHand()

        # lift grasped object
        rospy.loginfo("Controller.onGraspSearchCallback(): END")


if __name__ == '__main__':
    rospy.init_node("tubaf_grasping_controller", anonymous=False)
    cntrl = Controller()
    rospy.spin()
