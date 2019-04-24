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
import os
import copy

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import TransformListener


def pose_to_tf(pose, frame_name, parent_frame, time=None):
    """
    Generate a TF from a given pose, frame, and parent.
    """
    assert pose is not None, 'Cannot have None for pose.'
    tf = TransformStamped()
    tf.child_frame_id = frame_name
    if time is None:
        time = rospy.Time.now()
    tf.header.stamp = time
    tf.header.frame_id = parent_frame

    tf.transform.translation = pose.position
    tf.transform.rotation = pose.orientation

    return tf


class SmartEquipment:
    """
    Class for handling equipment on the robot with the manipulation unit, ie gripper
    """
    def __init__(self, entry={'name': "Empty", 'pose': {'frame': "/base_link",
                                                        'position':     {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                                        'orientation':  {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                              'mesh': {'pkg': "tbf_gripper_tools", 'path': ['resources', 'mesh', 'water_station.stl']},
                              'pickup_waypoints': {'pre_grasp': [], 'grasp': [], 'lift': [], 'post_grasp': []},
                              'place_waypoints': {'pre_set': [], 'set': [], 'lift': [], 'post_set': []},
                              'place_pose': {'frame': "/gripper_ur5_base_link",
                                             'position':    {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                             'oientation':  {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                              'hold_on_set': 0.0
                              }):
        """
        Default Constructor, may give an dict() with parameters
        :param entry: parameters
        :type entry: dict
        """
        self.name = entry["name"]
        self.ps = PoseStamped()
        self.ps.header.stamp = rospy.Time.now()
        self.ps.header.frame_id = entry["pose"]["frame"]
        self.ps.pose.position.x = entry["pose"]["position"]["x"]
        self.ps.pose.position.y = entry["pose"]["position"]["y"]
        self.ps.pose.position.z = entry["pose"]["position"]["z"]
        self.ps.pose.orientation.x = entry["pose"]["orientation"]["x"]
        self.ps.pose.orientation.y = entry["pose"]["orientation"]["y"]
        self.ps.pose.orientation.z = entry["pose"]["orientation"]["z"]
        self.ps.pose.orientation.w = entry["pose"]["orientation"]["w"]
        print rospkg.RosPack().get_path('rospy')
        self.mesh_path = os.path.join(rospkg.RosPack().get_path(entry['mesh']['pkg']), *entry['mesh']['path'])
        self.pickup_waypoints = entry["pickup_waypoints"]
        self.place_waypoints = entry["place_waypoints"]
        self.place_ps = PoseStamped()
        self.place_ps.header.stamp = rospy.Time.now()
        self.place_ps.header.frame_id = entry["place_pose"]["frame"]
        self.place_ps.pose.position.x = entry["place_pose"]["position"]["x"]
        self.place_ps.pose.position.y = entry["place_pose"]["position"]["y"]
        self.place_ps.pose.position.z = entry["place_pose"]["position"]["z"]
        self.place_ps.pose.orientation.x = entry["place_pose"]["orientation"]["x"]
        self.place_ps.pose.orientation.y = entry["place_pose"]["orientation"]["y"]
        self.place_ps.pose.orientation.z = entry["place_pose"]["orientation"]["z"]
        self.place_ps.pose.orientation.w = entry["place_pose"]["orientation"]["w"]
        self.hold_on_set = entry["hold_on_set"]
        self.grasp_offset = TransformStamped()

    def __str__(self):
        return self.name + \
               "\npose:\n" + str(self.ps) + \
               "\nhold_on_set: " + str(self.hold_on_set) + \
               "\nmesh_path: " + str(self.mesh_path) + \
               "\npickup_waypoints[pre]:\n" + str(self.pickup_waypoints["pre_grasp"]) + \
               "\npickup_waypoints[grasp]:\n" + str(self.pickup_waypoints["grasp"]) + \
               "\npickup_waypoints[post]:\n" + str(self.pickup_waypoints["post_grasp"])  + \
               "\npickup_waypoints[pre]:\n" + str(self.pickup_waypoints["pre_set"]) + \
               "\npickup_waypoints[grasp]:\n" + str(self.pickup_waypoints["set"]) + \
               "\npickup_waypoints[post]:\n" + str(self.pickup_waypoints["post_set"])

    def calculate_grasp_offset(self, attached_frame, planning_frame, tf_listener=None):
        """
        Calculates and stores the offset between the pick pose and the given frame
        :param planning_frame: Planning frame of the moveit planning group, see MoveItInterface
        :type planning_frame: str
        :param tf_listener: information about current transformations of the robot
        :type tf_listener: TransformListener
        :param attached_frame: typically the planinge frame from MoveIt
        :type attached_frame: str
        :return: -
        :rtype: -
        """
        if tf_listener is None:
            tf_listener = TransformListener()
        initial_equipment_pose = copy.deepcopy(self.ps)
        initial_equipment_pose.header.stamp = rospy.Time()

        tmp_frame = "temp_frame"

        eq_pose_transform = pose_to_tf(initial_equipment_pose.pose,tmp_frame, initial_equipment_pose.header.frame_id)
        # print("object_raw", eq_pose_transform)
        tf_listener.setTransform(eq_pose_transform)

        T_ZERO = rospy.Time()
        # print("planning frame: ", planning_frame)
        tf_listener.waitForTransform(attached_frame, tmp_frame, rospy.Time(), rospy.Duration.from_sec(5.0))
        ps = PoseStamped()
        ps.header.frame_id = tmp_frame
        ps.pose.orientation.w = 1.
        check_ps = tf_listener.transformPose(planning_frame, ps)
        # print("check_ps", check_ps)
        # print("object", tf_listener.lookupTransform(tmp_frame, planning_frame, T_ZERO))
        # print("hand  ", tf_listener.lookupTransform(attached_frame, planning_frame, T_ZERO))

        offset_t, offset_r = tf_listener.lookupTransform(tmp_frame,attached_frame,T_ZERO)

        r = self.grasp_offset.transform.rotation
        r.x, r.y, r.z, r.w = offset_r
        t = self.grasp_offset.transform.translation
        t.x, t.y, t.z = offset_t
        self.grasp_offset.header.frame_id = tmp_frame
        self.grasp_offset.child_frame_id = "temp_frame2"
        self.grasp_offset.header.stamp = eq_pose_transform.header.stamp

        # ps = PoseStamped()
        # ps.header.frame_id = "temp_frame2"
        # ps.pose.orientation.w = 1.
        # check_ps = tf_listener.transformPose(planning_frame, ps)
        # print("with offset: ", check_ps)

        rospy.logdebug("Equipment.calculate_grasp_offset(): offset:\n {}".format(self.grasp_offset))

    def get_grasp_pose(self, object_pose_stamped, planning_frame, tf_listener):
        """
        Temporarily add a frame to the tf listener and extract the grasp pose relative to the planning frame
        :param object_pose_stamped: current pose of the object
        :type object_pose_stamped: PoseStamped
        :param planning_frame: planning frame of the moveit group
        :type planning_frame: str
        :param tf_listener: tf interface
        :type tf_listener: TransformListener
        :return: grasp pose
        :rtype: PoseStamped
        """
        tmp_frame = "temp_frame"
        eq_pose_transform = pose_to_tf(object_pose_stamped.pose, tmp_frame, object_pose_stamped.header.frame_id)
        # print("object_raw", eq_pose_transform)
        tf_listener.setTransform(eq_pose_transform)
        self.grasp_offset.header.stamp = eq_pose_transform.header.stamp
        tf_listener.setTransform(self.grasp_offset)
        ps = PoseStamped()
        ps.header.frame_id = "temp_frame2"
        ps.pose.orientation.w = 1.
        check_ps = tf_listener.transformPose(planning_frame, ps)
        return check_ps
