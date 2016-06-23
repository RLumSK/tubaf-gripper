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

import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg

import grasping.haf_client
import grasping.hand
import grasping.arm

class Controller(object):
    # see: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

    def __init__(self):
        self.haf_client = grasping.haf_client.HAFClient()
        self.haf_client.add_grasp_cb_function(self.onGraspPointCallback)

        self.moveit_controller = grasping.arm.MoveItWrapper()

        self.hand_controller = grasping.hand.HandController()

        # Advertise the required topic
        self.planning_scene_diff_publisher = rospy.Publisher("planning_scene", moveit_msgs.msg.PlanningScene, queue_size=1)

        # Define the attached object message
        self.wlan_box = moveit_msgs.msg.AttachedCollisionObject()
        self.wlan_box.link_name = "wlan_box_link"
        self.wlan_box.object.header.frame_id = "tbd"
        self.wlan_box.object.id = "wlan_box_0"
        pose = geometry_msgs.msg.Pose()
        pose.orientation.w = 1.0
        primitive = shape_msgs.msg.SolidPrimitive
        primitive.type = primitive.BOX
        primitive.dimensions[0] = 1.0
        primitive.dimensions[2] = 1.0
        primitive.dimensions[3] = 1.0
        self.wlan_box.object.primitives.append(primitive)
        self.wlan_box.object.primitive_poses.append(pose)
        self.wlan_box.object.operation = self.wlan_box.object.ADD


    def onGraspPointCallback(self, point):
        # TODO: decide which point is worth grasping
        self.haf_client.remove_grasp_cb_function(self.onGraspPointCallback)
        self.hand_controller.openHand()

        # clear octomap at grasp point

        # Add an object into the environment
        rospy.loginfo("Controller.onGraspPointCallback(): Adding the object into the world at the location: %s", point)
        pl_scene = moveit_msgs.msg.PlanningScene()
        obj = moveit_msgs.msg.CollisionObject()
        obj.id = "box_0"
        obj.header = point.header
        obj.operation = obj.REMOVE
        rospy.loginfo("Controller.onGraspPointCallback(): Attaching the object to the hand and removing it from the world.")
        pl_scene.world.collision_objects.append(obj)
        pl_scene.robot_state.attached_collision_objects.append(obj)


        # plan path towards the object

        # may wait for approval

        # execute plan

        # grasp object
        self.hand_controller.closeHand()

        # lift grasped object


if __name__ == '__main__':
    rospy.init_node("tubaf_grasping_controller", anonymous=False)
    cntrl = Controller()
    rospy.spin()