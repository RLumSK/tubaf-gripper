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
import moveit_commander

import ar_track_alvar_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv

import numpy as np


class PlaningInterface(object):
    """
    class that transforms an ar pose of an marker on a known object into a collision object into the planning scene
    """
    def __init__(self):
        """
        Default constructor that loads parameter from the parameter server, register callbacks and publisher
        """
        rospy.loginfo("pl_interface.py:PlanningInterface() initializing")
        self.ar_topic = rospy.get_param("~ar_topic", "/ar_pose_marker")
        self.ar_sub = rospy.Subscriber(self.ar_topic, ar_track_alvar_msgs.msg.AlvarMarkers, callback=self.onMarkersMessage,
                                       queue_size=1)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.marker = None

        self.stl = rospy.get_param("~model_path", "package://tbf_gripper_perception/meshes/wlan_box.stl")
        self.collision_scale = rospy.get_param("~model_scale", 1e-03)

        # http://answers.ros.org/question/157716/obstacles-management-in-moveit/
        self._pubPlanningScene = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene, queue_size=10)
        rospy.wait_for_service('/get_planning_scene', 10.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', moveit_msgs.srv.GetPlanningScene)
        request = moveit_msgs.msg.PlanningSceneComponents(
            components=moveit_msgs.msg.PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = get_planning_scene(request)
        allowed_collisions_matrix = response.scene.allowed_collision_matrix

        # allow collision
        if self.ar_topic not in allowed_collisions_matrix.default_entry_names:
            rospy.loginfo("pl_interface.py:PlanningInterface() add object to allowed collision matrix")
            # add button to allowed collision matrix
            # allowed_collisions_matrix.entry_values
            allowed_collisions_matrix.default_entry_names += [self.ar_topic]
            allowed_collisions_matrix.default_entry_values += [True]

            planning_scene_diff = moveit_msgs.msg.PlanningScene(
                is_diff=True,
                allowed_collision_matrix=allowed_collisions_matrix)

            self._pubPlanningScene.publish(planning_scene_diff)
            rospy.sleep(1.0)
        rospy.loginfo("pl_interface.py:PlanningInterface() initialized")

    def onMarkersMessage(self, msg):
        """
        After receiving a set of markers publish the referring collision object in the planning scene
        :param msg: message from the ar_track_alvar package
        :type msg: ar_track_alvar_msgs.msg.AlvarMarkers
        :return: -
        :rtype: -
        """
        # rospy.loginfo("pl_interface.py:PlanningInterface.onMarkersMessage() received msg:%s", msg)
        if len(msg.markers) < 1:
            return
        max_marker = msg.markers[0]
        # determine which pose gone serve as reference for the object
        # for i in range(1, len(msg.markers)):
        #     if msg.markers[i].confidence > confidence:
        #         max_marker = msg.markers[i]
        #         confidence = msg.markers[i].confidence
        # if max_marker is None:
        #     return

        # rospy.loginfo("pl_interface.py:PlanningInterface.onMarkersMessage() max marker:%s", max_marker)
        # add offset to ar_marker pose
        max_marker.pose.pose.position.z -= 0.078
        max_marker.pose.header = max_marker.header

        # determine if pose has changed
        if self.marker is None:
            self.marker = max_marker
        else:
            if self.poses_match(self.marker.pose.pose, max_marker.pose.pose):
                # posese seam simlar
                pass
            else:
                self.marker = max_marker
                self.scene.remove_world_object(name=self.ar_topic)
                rospy.sleep(0.2)
                # add collision object at given pose
                cs = self.collision_scale
                self.scene.add_mesh(name=self.ar_topic, pose= self.marker.pose, filename=self.stl, size=(cs, cs, cs))
        rospy.loginfo("pl_interface.py:PlanningInterface.onMarkersMessage() finished")
        rospy.sleep(1.0)

    def poses_match(self, pose1, pose2, pos_tol=0.005, rot_tol=1.0):
        """
        Determine if two given poses are similar or not by comparing position and orientation
        :param pose1: 1st pose
        :type pose1: Pose
        :param pose2: 2nd pose
        :type pose2: Pose
        :param pos_tol: position tolerance, default: 0.005
        :type pos_tol: Double
        :param rot_tol: rotation tolerance, default: 1.0
        :type rot_tol: Double
        :return: True if similar, False if not
        :rtype: Boolean
        """
        pos1 = np.array((pose1.position.x, pose1.position.y, pose1.position.z))
        pos2 = np.array((pose2.position.x, pose2.position.y, pose2.position.z))
        dp = np.linalg.norm(pos2-pos1)

        q1 = (pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w)
        q2 = (pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w)
        dq = 1 - np.abs(np.dot(q2, q1))

        return dp <= pos_tol and dq <= rot_tol

if __name__ == '__main__':
    rospy.init_node("tbf_gripper_perception_planing_interface", anonymous=False)
    obj = PlaningInterface()

    rospy.spin()
