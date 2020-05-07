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

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import TransformListener,  transformations as tft
import numpy as np


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
        self.grasp_offset = np.eye(4)  # type: np.ndarray   # description: Affine transformation from mesh origin to
                                                            # gripper pose

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
        for entry in rospy.get_param(group_name):
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
               "\npickup_waypoints[post]:\n" + str(self.pickup_waypoints["post_grasp"])  + \
               "\nplace_waypoints[pre]:\n" + str(self.place_waypoints["pre_set"]) + \
               "\nplace_waypoints[set]:\n" + str(self.place_waypoints["set"]) + \
               "\nplace_waypoints[post]:\n" + str(self.place_waypoints["post_set"])

    def calculate_grasp_offset(self, attached_frame="gripper_robotiq_palm_planning", tf_listener=None):
        """
        Calculates and stores the offset between the pick pose and the given frame as PoseStamped
        :param attached_frame: typically the end effector frame
        :type attached_frame: str
        :param tf_listener: information about current transformations of the robot
        :type tf_listener: TransformListener
        :return: -
        :rtype: -
        """
        # Transform all poses in one coordinate frame
        if tf_listener is None:
            tf_listener = TransformListener()
        self.ps.header.stamp = rospy.Time.now()
        tf_listener.waitForTransform(target_frame=self.ps.header.frame_id, source_frame=attached_frame,
                                     time=rospy.Time.now(), timeout=rospy.Duration.from_sec(2.0))
        bare_gripper_pose = PoseStamped()
        bare_gripper_pose.header.frame_id = attached_frame
        bare_gripper_pose.header.stamp = rospy.Time.now()
        gripper_pose = None
        while gripper_pose is None:
            try:
                gripper_pose = tf_listener.transformPose(target_frame=self.ps.header.frame_id, ps=bare_gripper_pose)
                if self.grasp_offset is None:
                    rospy.sleep(2.0)
            except Exception as ex:
                rospy.logwarn("SmartEquipment.calculate_grasp_offset(): %s", ex)
        # rospy.logdebug("SmartEquipment.calculate_grasp_offset(): gripper_pose:\n%s", gripper_pose)
        # Both Pose are now in the same coordinate Frame
        # Calculate Offset as the missing connection(O) between gripper (G) and smart equipment pose (E)
        # G = E*O --> E^-1*G = O
        # https://answers.ros.org/question/215656/how-to-transform-a-pose/
        E_T = tft.translation_matrix([self.ps.pose.position.x, self.ps.pose.position.y, self.ps.pose.position.z])
        E_R = tft.quaternion_matrix([self.ps.pose.orientation.x, self.ps.pose.orientation.y, self.ps.pose.orientation.z,
                                     self.ps.pose.orientation.w])
        E = np.dot(E_T, E_R)

        G_T = tft.translation_matrix([gripper_pose.pose.position.x, gripper_pose.pose.position.y,
                                      gripper_pose.pose.position.z])
        G_R = tft.quaternion_matrix([gripper_pose.pose.orientation.x, gripper_pose.pose.orientation.y,
                                     gripper_pose.pose.orientation.z, gripper_pose.pose.orientation.w])
        G = np.dot(G_T, G_R)

        O = np.dot(np.linalg.inv(E), G)
        self.grasp_offset = O
        # O_T = tft.translation_from_matrix(O)
        # O_R = tft.quaternion_from_matrix(O)
        #
        # self.grasp_offset = PoseStamped()
        # self.grasp_offset.header = gripper_pose.header
        # self.grasp_offset.pose.position.x, self.grasp_offset.pose.position.y, self.grasp_offset.pose.position.z = O_T
        # self.grasp_offset.pose.orientation.x, self.grasp_offset.pose.orientation.y, \
        # self.grasp_offset.pose.orientation.z, self.grasp_offset.pose.orientation.w = O_R
        # rospy.logdebug("SmartEquipment.calculate_grasp_offset(): offset:\n {}".format(self.grasp_offset))

    def get_grasp_pose(self, object_pose_stamped, tf_listener=None):
        """
        Calculating the current gripper pose, based on the saved offset and given object pose
        :param object_pose_stamped: current pose of the object
        :type object_pose_stamped: PoseStamped
        :param tf_listener: Transformation Listener from the ROS tf library
        :type tf_listener: TransformListener
        :return: grasp pose
        :rtype: PoseStamped
        """
        if np.array_equal(self.grasp_offset, np.eye(4)):
            rospy.logdebug("SmartEquipment.get_grasp_pose(): grasp_offset is Zero")
            self.calculate_grasp_offset(attached_frame="gripper_robotiq_palm_planning", tf_listener=tf_listener)
        # rospy.logdebug("SmartEquipment.get_grasp_pose(): Saved Transformation is: %s", self.grasp_offset)

        op_trans_mat = tft.translation_matrix([object_pose_stamped.pose.position.x,
                                               object_pose_stamped.pose.position.y,
                                               object_pose_stamped.pose.position.z])
        op_rot_mat = tft.quaternion_matrix([object_pose_stamped.pose.orientation.x,
                                            object_pose_stamped.pose.orientation.y,
                                            object_pose_stamped.pose.orientation.z,
                                            object_pose_stamped.pose.orientation.w])
        op_mat = np.dot(op_trans_mat, op_rot_mat)

        ret_mat = np.dot(op_mat, self.grasp_offset)

        ret_rot = tft.quaternion_from_matrix(ret_mat)
        ret_trans = tft.translation_from_matrix(ret_mat)

        ps = PoseStamped()
        ps.header = object_pose_stamped.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = ret_trans
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = ret_rot
        # rospy.logdebug("SmartEquipment.get_grasp_pose(): Calculated Pose:\n%s", ps)
        return ps


if __name__ == '__main__':
    rospy.init_node("SmartEquipment", log_level=rospy.INFO)
    # Equipment Parameter
    for equip in rospy.get_param("~smart_equipment"):
        eq = SmartEquipment(equip)
        rospy.loginfo("[SmartEquipment] %s", eq)
