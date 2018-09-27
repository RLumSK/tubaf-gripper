#!/usr/bin/python
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, TU Bergakademie Freiberg
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
import resource_retriever as rer
from geometry_msgs.msg import PoseStamped, Point, Pose
from tf import TransformListener


class Equipment:
    """
    Class for handling equipment on the robot with the manipulation unit, ie gripper
    """
    def __init__(self, entry={'name': "Empty", 'robot_pick_pose': {'reference_frame': "/base_link",
                                                                   'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                                                   'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0,
                                                                                   'w': 1.0}},
                              'release_on_set': True, 'mesh_url': "", "scale": {'x': 1.0, 'y': 1.0, 'z': 1.0},
                              'pick_waypoints': {'pre_joint_values': [], 'grasp_joint_values': [],
                                                 'post_joint_values': []}}):
        """
        Default Constructor, may give an dict(9 with parameters
        :param entry: parameters
        :type entry: dict
        """
        self.name = entry["name"]
        self.robot_pick_pose = PoseStamped()
        self.robot_pick_pose.header.stamp = rospy.Time.now()
        self.robot_pick_pose.header.frame_id = entry["robot_pick_pose"]["reference_frame"]
        self.robot_pick_pose.pose.position.x = entry["robot_pick_pose"]["position"]["x"]
        self.robot_pick_pose.pose.position.y = entry["robot_pick_pose"]["position"]["y"]
        self.robot_pick_pose.pose.position.z = entry["robot_pick_pose"]["position"]["z"]
        self.robot_pick_pose.pose.orientation.x = entry["robot_pick_pose"]["orientation"]["x"]
        self.robot_pick_pose.pose.orientation.y = entry["robot_pick_pose"]["orientation"]["y"]
        self.robot_pick_pose.pose.orientation.z = entry["robot_pick_pose"]["orientation"]["z"]
        self.robot_pick_pose.pose.orientation.w = entry["robot_pick_pose"]["orientation"]["w"]
        self.release_on_set = entry["release_on_set"]
        self.mesh_path = rer.get_filename(entry["mesh_url"])[7:]
        self.scale = Point()
        self.scale.x = entry["scale"]["x"]
        self.scale.y = entry["scale"]["y"]
        self.scale.z = entry["scale"]["z"]
        self.pick_waypoints = dict()
        self.pick_waypoints["pre"] = entry["pick_waypoints"]["pre_joint_values"]
        self.pick_waypoints["grasp"] = entry["pick_waypoints"]["grasp_joint_values"]
        self.pick_waypoints["post"] = entry["pick_waypoints"]["post_joint_values"]
        self.touch_links = entry["touch_links"]
        if self.touch_links[0] == "":
            self.touch_links = []
        self.grasp_offset = PoseStamped()

    def __str__(self):
        return self.name + \
               "\nrobot_pick_pose:\n" + str(self.robot_pick_pose) + \
               "\nrelease_on_set: " + str(self.release_on_set) + \
               "\nmesh_path: " + str(self.mesh_path) + \
               "\nscale:\n" + str(self.scale) + \
               "\npick_waypoints[pre]:\n" + str(self.pick_waypoints["pre"]) + \
               "\npick_waypoints[grasp]:\n" + str(self.pick_waypoints["grasp"]) + \
               "\npick_waypoints[post]:\n" + str(self.pick_waypoints["post"])

    def calculate_grasp_offset(self, attached_frame="gripper_robotiq_palm_planning", tf_listener=None):
        """
        Calculates and stores the offset between the pick pose and the given frame
        :param tf_listener: information about current transformations of the robot
        :type tf_listener: TransformListener
        :param attached_frame: typically the planinge frame from MoveIt
        :type attached_frame: str
        :return: -
        :rtype: -
        """
        if tf_listener is None:
            tf_listener = TransformListener()
        self.robot_pick_pose.header.stamp = rospy.Time()
        tf_listener.waitForTransform(attached_frame, self.robot_pick_pose.header.frame_id,
                                     self.robot_pick_pose.header.stamp, rospy.Duration.from_sec(5.0))
        self.grasp_offset = tf_listener.transformPose(attached_frame, self.robot_pick_pose)
        rospy.loginfo("Equipment.calculate_grasp_offset(): offset:\n %s" % self.grasp_offset)
