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
from tf import TransformListener, transformations as tft
import numpy as np
from tubaf_tools.help import array_to_pose, pose_to_array


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
                                                        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                              'mesh': {'pkg': "tbf_gripper_tools", 'path': ['resources', 'mesh', 'water_station.stl']},
                              'pickup_waypoints': {'pre_grasp': [], 'grasp': [], 'lift': [], 'post_grasp': []},
                              'place_waypoints': {'pre_set': [], 'set': [], 'lift': [], 'post_set': []},
                              'place_pose': {'frame': "/gripper_ur5_base_link",
                                             'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                             'oientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                              'hold_on_set': 0.0
                              }):
        """
        Default Constructor, may give an dict() with parameters
        :param entry: parameters
        :type entry: dict
        """
        self.name = entry["name"]
        self.ps = PoseStamped()  # type: PoseStamped
        self.ps.header.stamp = rospy.Time.now()
        self.ps.header.frame_id = entry["pose"]["frame"]
        self.ps.pose.position.x = entry["pose"]["position"]["x"]
        self.ps.pose.position.y = entry["pose"]["position"]["y"]
        self.ps.pose.position.z = entry["pose"]["position"]["z"]
        self.ps.pose.orientation.x = entry["pose"]["orientation"]["x"]
        self.ps.pose.orientation.y = entry["pose"]["orientation"]["y"]
        self.ps.pose.orientation.z = entry["pose"]["orientation"]["z"]
        self.ps.pose.orientation.w = entry["pose"]["orientation"]["w"]
        self.mesh_pkg = entry['mesh']['pkg']
        self.mesh_rel_path = os.path.join(*entry['mesh']['path'])
        self.mesh_path = os.path.join(rospkg.RosPack().get_path(self.mesh_pkg), *entry['mesh']['path'])
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
        self.ssb_T_gripper = np.eye(4)  # type: np.ndarray   # description: Affine transformation from mesh origin to
        # gripper pose
        # rospy.logwarn("[SmartEquipment.__init__] %s has mesh-path: %s" % (self.name, self.mesh_path))

    @classmethod
    def from_parameter_server(cls, group_name="~smart_equipment"):
        """
        Constructing a list with smart equipment, assuming the needed parameters are available on the ROS parameter
        server
        :param group_name: identifier on the parameter server
        :type group_name: str
        :return: list
        :rtype: list
        """
        ret_lst = []  # type: list
        for entry in rospy.get_param(group_name, default=[]):
            # rospy.logdebug("%s", entry)
            ret_lst.append(SmartEquipment(entry))
        return ret_lst

    def __str__(self):
        return self.name + \
               "\npose:\n" + str(self.ps) + \
               "\nhold_on_set: " + str(self.hold_on_set) + \
               "\nmesh_path: " + str(self.mesh_path) + \
               "\npickup_waypoints[pre]:\n" + str(self.pickup_waypoints["pre_grasp"]) + \
               "\npickup_waypoints[grasp]:\n" + str(self.pickup_waypoints["grasp"]) + \
               "\npickup_waypoints[post]:\n" + str(self.pickup_waypoints["post_grasp"]) + \
               "\nplace_waypoints[pre]:\n" + str(self.place_waypoints["pre_set"]) + \
               "\nplace_waypoints[set]:\n" + str(self.place_waypoints["set"]) + \
               "\nplace_waypoints[post]:\n" + str(self.place_waypoints["post_set"])

    def calculate_transformations(self, gripper_frame="gripper_robotiq_palm_planning", tf_listener=None):
        """
        Calculates and stores the offset between the current pose and the given frame as PoseStamped
        :param gripper_frame: typically the end effector frame
        :type gripper_frame: str
        :param tf_listener: information about current transformations of the robot
        :type tf_listener: TransformListener
        :return: tuple with transformations [base_T_ssb, ssb_T_gripper, base_T_gripper]
        :rtype: tuple of affine transformations
        """
        if tf_listener is None:
            tf_listener = TransformListener()
        tf_listener.waitForTransform(target_frame=self.ps.header.frame_id, source_frame=gripper_frame,
                                     time=rospy.Time(0), timeout=rospy.Duration.from_sec(2.0))
        bare_gripper_pose = PoseStamped()
        bare_gripper_pose.header.frame_id = gripper_frame
        bare_gripper_pose.header.stamp = rospy.Time(0)  # rospy.Time.now()
        ps_base_to_gripper = None
        while ps_base_to_gripper is None:
            try:
                ps_base_to_gripper = tf_listener.transformPose(target_frame=self.ps.header.frame_id,
                                                               ps=bare_gripper_pose)
            except Exception as ex:
                rospy.logwarn("SmartEquipment.calculate_transformations(): %s", ex)

        # Transform it into affine transformation
        base_Tt_gripper = tft.translation_matrix([ps_base_to_gripper.pose.position.x,
                                                  ps_base_to_gripper.pose.position.y,
                                                  ps_base_to_gripper.pose.position.z])
        base_Tr_gripper = tft.quaternion_matrix([ps_base_to_gripper.pose.orientation.x,
                                                 ps_base_to_gripper.pose.orientation.y,
                                                 ps_base_to_gripper.pose.orientation.z,
                                                 ps_base_to_gripper.pose.orientation.w])
        base_T_gripper = np.dot(base_Tt_gripper, base_Tr_gripper)

        base_Tt_ssb = tft.translation_matrix([self.ps.pose.position.x,
                                              self.ps.pose.position.y,
                                              self.ps.pose.position.z])
        base_Tr_ssb = tft.quaternion_matrix([self.ps.pose.orientation.x,
                                             self.ps.pose.orientation.y,
                                             self.ps.pose.orientation.z,
                                             self.ps.pose.orientation.w])
        base_T_ssb = np.dot(base_Tt_ssb, base_Tr_ssb)

        # base_T_ssb * ssb_T_gripper = base_T_gripper -> ssb_T_greifer = base_T_ssb^-1 * base_T_gripper

        ssb_T_gripper = np.dot(np.linalg.inv(base_T_ssb), base_T_gripper)
        rospy.logdebug("SmartEquipment.calculate_transformations(): ssb_T_gripper=?\n%s" % ssb_T_gripper)
        return [base_T_ssb, ssb_T_gripper, base_T_gripper]

    def get_grasp_pose(self, object_pose_stamped=None, tf_listener=None, gripper_frame="gripper_robotiq_palm_planning",
                       save_relation=False, use_relation=False):
        """
        Calculating the current gripper pose relative to a given object pose
        :param use_relation: whether or not to use a previous calculated ssb_T_gripper
        :param save_relation: whether or not to preserve the relative transformation ssb_T_gripper as class variable
        :param gripper_frame: reference coordinate frame of the gripper
        :param object_pose_stamped: current pose of the object
        :type object_pose_stamped: PoseStamped
        :param tf_listener: Transformation Listener from the ROS tf library
        :type tf_listener: TransformListener
        :return: grasp pose
        :rtype: PoseStamped
        """
        # base_T_ssb * ssb_T_gripper = base_T_gripper -> ssb_T_greifer = base_T_ssb^-1 * base_T_gripper
        base_T_ssb, ssb_T_gripper, base_T_gripper = self.calculate_transformations(gripper_frame=gripper_frame,
                                                                                   tf_listener=tf_listener)
        if use_relation:
            ssb_T_gripper = self.ssb_T_gripper
        if save_relation:
            self.ssb_T_gripper = ssb_T_gripper

        if object_pose_stamped is not None:
            if tf_listener is None:
                tf_listener = TransformListener(rospy.Duration.from_sec(15.0))
            tf_listener.waitForTransform(self.ps.header.frame_id,
                                         object_pose_stamped.header.frame_id,
                                         object_pose_stamped.header.stamp,
                                         rospy.Duration(4))
            object_pose_stamped = tf_listener.transformPose(self.ps.header.frame_id, object_pose_stamped)
        else:
            object_pose_stamped = self.ps
        gp = copy.deepcopy(object_pose_stamped)
        gp.header.frame_id = self.name
        gp.pose = array_to_pose(ssb_T_gripper)
        rospy.logdebug("SmartEquipment.get_grasp_pose(): Calculated Pose:\n%s", gp)
        return gp


if __name__ == '__main__':
    rospy.init_node("SmartEquipment", log_level=rospy.INFO)
    # Equipment Parameter
    for equip in rospy.get_param("~smart_equipment"):
        eq = SmartEquipment(equip)
        rospy.loginfo("[SmartEquipment] %s", eq)
