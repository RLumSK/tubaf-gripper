#!/usr/bin/python
# coding=utf-8
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
# Author: grehl
import copy

import rospy
import tf
import message_filters
import numpy as np

from geometry_msgs.msg import PoseStamped
import std_msgs.msg

from autonomy.Task import GraspTask
from autonomy.MoveitInterface import MoveitInterface
from tbf_gripper_tools.SmartEquipment import SmartEquipment
from tubaf_tools import rotate_pose, array_from_xyzrpy, array_to_pose, pose_to_array
import tbf_gripper_rviz.ssb_marker as marker
from tf import TransformListener


def continue_by_topic(topic=None):
    """
    Wait for a msg from a defined topic
    :param topic: name of the topic where the msg will be received
    :type topic: String
    :return: result
    :rtype: Bool
    """
    if topic is None:
        topic = rospy.get_param("~continue_topic", "/equipment_task/continue")
    subscriber = message_filters.Subscriber(topic, std_msgs.msg.Bool)
    cache = message_filters.Cache(subscriber, 5, allow_headerless=True)
    while cache.getLast() is None:
        rospy.sleep(2.0)
    return cache.getLast().data


def sense(tf_listener=None):
    """
    Sense for a suitable pose in a set of clusters in a plane
    :return: determined pose
    :rtype: PoseStamped
    """
    from tbf_gripper_autonomy.srv import GenerateSetPose, GenerateSetPoseRequest
    from std_msgs.msg import Header
    from message_filters import Subscriber, Cache
    from object_recognition_msgs.msg import TableArray
    from visualization_msgs.msg import MarkerArray

    # As client
    # use: PcaPoseGenerator, MinimalDensityEstimatePoseGenerator, DelaunayPoseGenerator
    service_name = rospy.get_param("~sense_service_name", "PcaPoseGenerator")
    rospy.wait_for_service(service_name + '_service')
    service = rospy.ServiceProxy(service_name + '_service', GenerateSetPose)

    request = GenerateSetPoseRequest()
    request.header = Header()
    request.header.stamp = rospy.Time.now()
    request.print_evaluation = False
    request.policy = "hl"

    _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
    _floor_topic = rospy.get_param("~floor_topic", "/ork/floor_plane")

    _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)
    _floor_cache = Cache(Subscriber(_floor_topic, TableArray), 1)

    ps = None

    while ps is None:
        try:
            request.floor = _floor_cache.getLast()
            request.obstacles = _obstacle_cache.getLast()
            if request.floor is None or request.obstacles is None:
                rospy.sleep(1.0)
                continue
            rospy.logdebug("[sense()] Request:\n%s" % request)
            reply = service(request)
            rospy.loginfo("[sense()] %s suggests %s" % (service_name, reply))

            ps = reply.set_pose

        except rospy.ServiceException as e:
            rospy.logerr("[main] Service %s call failed\n%s" % (service_name, e.message))
            ps = None

    # Adjust z-coordinate to fit to ground
    if tf_listener is None:
        tf_listener = TransformListener(rospy.Duration.from_sec(15.0))
    bf = "base_footprint"
    tf_listener.waitForTransform(bf, ps.header.frame_id, rospy.Time(0), rospy.Duration.from_sec(15.0))
    ret_ps = tf_listener.transformPose(bf, ps)
    ret_ps.pose.position.z = 0
    return ret_ps


def adjust_z_rotation(ps_object):
    """
    Adjust the z-Rotation of the given pose in order to be easier to reach for the robot
    :param ps_object: pose to adjust
    :type ps_object: PoseStamped
    :return: adjusted pose
    :rtype: PoseStamped
    """
    ret_ps = PoseStamped()
    ret_ps.header = ps_object.header

    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_dcm(pose_to_array(ps_object.pose)[:3, :3])  # type: R
    old = rotation.as_euler('XYZ')
    new = np.copy(old)
    rospy.logdebug("adjust_z_rotation(): alpha %s" % old[0])
    rospy.logdebug("adjust_z_rotation(): beta %s" % old[1])
    rospy.logdebug("adjust_z_rotation(): gamma %s" % old[2])

    # Formula
    # frac = (2+np.cos(beta))/(1+np.cos(beta))
    # rospy.loginfo("adjust_z_rotation(): arccos(%s)" % frac)
    # new[2] = old[0] #new[2] - np.pi/2.0

    # Compute change
    d = new - old
    for i in range(0, 2):
        if d[i] > np.pi:
            d[i] -= 2 * np.pi
        if d[i] < -np.pi:
            d[i] += 2 * np.pi

    ret_ps.pose = rotate_pose(ps_object.pose, d)
    rospy.logdebug("adjust_z_rotation(): dalpha %s" % d[0])
    rospy.logdebug("adjust_z_rotation(): dbeta %s" % d[1])
    rospy.logdebug("adjust_z_rotation(): dgamma %s" % d[2])
    return ret_ps


def optimize_ssb_z_rotation(oTs, sTg):
    """
    Calculate the missing z-Rotation of transformation T from O->S, given the transformation and S->G
    O ... base coordinate frame, eg. base_link
    S ... SSB coordinate frame
    G ... gripper coordinate frame
    The rotation among the z-axis for oTs is variable.
    Therefore we want to minimize otg, this yields to:
    ots = -oRs*stg
    t ... translation
    R ... rotation matrix
    :param oTs: transformation from O->S as affine transformation
    :type oTs: numpy.ndarray (4x4)
    :param sTg: transformation from S->G as affine transformation
    :type sTg: numpy.ndarray (4x4)
    :return: transformation from O->S
    :rtype: numpy.ndarray (4x4)
    """
    # Setup variables
    ots = oTs[:3, 3]
    stg = sTg[:3, 3]
    # oRs = Rz*Ry*Rx
    from scipy.spatial.transform import Rotation
    R = Rotation.from_dcm(oTs[:3, :3])
    alpha, beta, gamma = R.as_euler('xyz')
    # Extrinsic rotation using euler angles
    Rx = np.asarray([[1, 0, 0],
                     [0, np.cos(alpha), np.sin(alpha)],
                     [0, -np.sin(alpha), np.cos(alpha)]])
    Ry = np.asarray([[np.cos(beta), 0, np.sin(beta)],
                     [0, 1, 0],
                     [-np.sin(beta), 0, np.cos(beta)]])
    # 1st compute the right side (known values)
    # Rx, Ry are orthogonal -> R'=R^-1 (AB)'=B'A'
    B = (-1.0 / np.linalg.norm(stg)) * np.dot(ots, stg.transpose()) * np.matmul(Ry, Rx).transpose()

    # 2nd extract gamma from B and calculate Rz
    b_alpha, b_beta, b_gamma = Rotation.from_dcm(B).as_euler('xyz')
    if b_gamma > 0:
        b_gamma -= np.pi
    else:
        b_gamma += np.pi
    Rz = np.asarray([[np.cos(b_gamma), np.sin(b_gamma), 0],
                     [-np.sin(b_gamma), np.cos(b_gamma), 0],
                     [0, 0, 1]])

    rospy.logdebug("[optimize_ssb_z_rotation()] B \n%s\ndet(B)=%s\tgamma=%s" % (
        B, np.linalg.det(B), Rotation.from_dcm(B).as_euler('xyz')[2]))
    rospy.logdebug("[optimize_ssb_z_rotation()] Rz \n%s\ndet(Rz)=%s\tgamma=%s" % (
        Rz, np.linalg.det(Rz), Rotation.from_dcm(Rz).as_euler('xyz')[2]))
    oTs[:3, :3] = np.matmul(Rz, np.matmul(Rx, Ry))
    # oTs[:3, :3] = np.matmul(B, np.matmul(Rx, Ry))
    rospy.logdebug("[optimize_ssb_z_rotation()] new oTs\n%s" % oTs)
    return oTs


def optimize_ssb_z_rotation2(oTs, sTg):
    """
    Calculate the missing z-Rotation of transformation T from O->S, given the transformation and S->G
    O ... base coordinate frame, eg. base_link
    S ... SSB coordinate frame
    G ... gripper coordinate frame
    The rotation among the z-axis for oTs is variable.
    Therefore we want to minimize otg, this yields to:
    ots = -oRs*stg
    t ... translation
    R ... rotation matrix
    :param oTs: transformation from O->S as affine transformation
    :type oTs: numpy.ndarray (4x4)
    :param sTg: transformation from S->G as affine transformation
    :type sTg: numpy.ndarray (4x4)
    :return: transformation from O->S
    :rtype: numpy.ndarray (4x4)
    """
    # Setup variables
    from scipy.spatial.transform import Rotation
    R = Rotation.from_dcm(oTs[:3, :3])
    alpha, beta, gamma = R.as_euler('xyz')
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    cb = np.cos(beta)

    a_s = -sa * (1 + cb)
    a_c = ca * (1 - cb)

    gamma = np.arccos((1 - cb) / np.sqrt(a_s ** 2 + a_c ** 2)) - np.arctan(-a_s / a_c)
    rospy.logdebug("[optimize_ssb_z_rotation()] gamma = %s°" % np.rad2deg(gamma))
    n_wRs = Rotation.from_euler('XYZ', [alpha, beta, gamma], degrees=False)
    oTs[:3, :3] = n_wRs.as_dcm()
    return oTs


def optimize_ssb_z_rotation3(oTs, sTg):
    """
    Calculate the missing z-Rotation of transformation T from O->S, given the transformation and S->G
    O ... base coordinate frame, eg. base_link
    S ... SSB coordinate frame
    G ... gripper coordinate frame
    The rotation among the z-axis for oTs is variable.
    Therefore we want to minimize otg, this yields to:
    ots = -oRs*stg
    t ... translation
    R ... rotation matrix
    :param oTs: transformation from O->S as affine transformation
    :type oTs: numpy.ndarray (4x4)
    :param sTg: transformation from S->G as affine transformation
    :type sTg: numpy.ndarray (4x4)
    :return: transformation from O->S
    :rtype: numpy.ndarray (4x4)
    """
    # Setup variables
    from scipy.spatial.transform import Rotation
    R = Rotation.from_dcm(oTs[:3, :3])
    alpha, beta, gamma = R.as_euler('xyz')

    # Newton Verfahren
    # stg_x * sin(gamma) + stg_y * cos(gamma) = 0, gesucht gamma
    def f_df_ddf(a, b, g, sph, aps):
        assert (len(sph) == 3)
        assert (len(aps) == 3)
        sa = np.sin(a)
        ca = np.cos(a)
        sb = np.sin(b)
        cb = np.cos(b)
        sg = np.sin(g)
        cg = np.cos(g)

        px = sph[0]
        py = sph[1]
        pz = sph[2]

        qx = aps[0]
        qy = aps[1]
        qz = aps[2]

        r00 = ca * cg - sa * cb * sg
        r01 = -ca * sg - sa * cb * cg
        r02 = sa * sb
        r10 = sa * cg + ca * cb * sg
        r11 = -sa * sg + ca * cb * cg
        r12 = -ca * sb
        r20 = sb * sg
        r21 = sb * cg
        r22 = cb

        dr00 = -ca * sg - sa * cb * cg
        dr01 = -ca * cg + sa * cb * sg
        dr10 = -sa * sg + ca * cb * cg
        dr11 = -sa * cg - ca * cb * sg
        dr20 = sb * cg
        dr21 = -sb * sg

        ddr00 = -ca * cg - sa * cb * sg
        ddr01 = ca * sg + sa * cb * cg
        ddr10 = -sa * cg - ca * cb * sg
        ddr11 = sa * sg - ca * cb * cg
        ddr20 = -sb * sg
        ddr21 = -sb * cg

        x = r00 * px + r01 * py + r02 * pz + qx
        dx = dr00 * px + dr01 * py
        ddx = ddr00 * px + ddr01 * py

        y = r10 * px + r11 * py + r12 * pz + qy
        dy = dr10 * px + dr11 * py
        ddy = ddr10 * px + ddr11 * py

        z = r20 * px + r21 * py + r22 * pz + qz
        dz = dr20 * px + dr21 * py
        ddz = ddr20 * px + ddr21 * py

        f = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        df = (x * dx + y * dy + z * dz) / f
        ddf = ((dx ** 2 + x * ddx + dy ** 2 + y * ddy + dz ** 2 + z * ddz) - df ** 2) / (f)

        return [f, df, ddf]

    def _newton(a, b, g, sph, aps, e):
        x = g
        lx = 2 * (x + e)
        ddf = -1
        while np.abs(x - lx) > e:
            [f, df, ddf] = f_df_ddf(a, b, x, sph, aps)
            lx = x
            x = x - (df / ddf)
        if ddf <= 0:
            return _newton(a, b, g+np.pi*0.3, sph, aps, e)
        return x % (2 * np.pi)

    gamma = _newton(alpha, beta, gamma, sTg[:3, 3], oTs[:3, 3], 1e-7)
    rospy.logdebug("optimize_ssb_z_rotation3() gamma = %s°" % np.rad2deg(gamma))
    n_wRs = Rotation.from_euler('XYZ', [alpha, beta, gamma], degrees=False)
    oTs[:3, :3] = n_wRs.as_dcm()
    return oTs


class EquipmentTask(GraspTask):
    """
    Handle equipment using the gripper unit of Julius
    """

    def __init__(self):
        """
        Default constructor, start ROS, hand_model and demo_monitoring
        """
        self.tf_listener = tf.TransformListener(rospy.Duration.from_sec(15.0))
        # Init Moveit
        self.moveit = MoveitInterface("~moveit", self.tf_listener)  # type: MoveitInterface
        rospy.loginfo("EquipmentTask.__init__(): Moveit initialized")
        # Equipment Parameter
        self.lst_equipment = SmartEquipment.from_parameter_server(group_name="~smart_equipment")
        for eq in self.lst_equipment:
            self.moveit.add_equipment(eq)
        self.selected_equipment = self.lst_equipment[0]  # type: SmartEquipment
        rospy.loginfo("EquipmentTask.__init__(): Equipment initialized")

        # Static joint values for specific well known poses
        self.backup_joint_values = rospy.get_param("~arm/backup_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.home_joint_values = rospy.get_param("~arm/home_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.watch_joint_values = rospy.get_param("~arm/watch_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.env_sense_joints_poses = rospy.get_param("~arm/env_scan_joint_poses", [])

        # @All parameters were imported@
        super(EquipmentTask, self).__init__(js_t=rospy.get_param("~arm/joint_states_topic"),
                                            bu_pos=self.backup_joint_values,
                                            ltcp_s=rospy.get_param("~arm/linear_tcp_speed"),
                                            ltcp_a=rospy.get_param("~arm/linear_tcp_acceleration"),
                                            j_s=rospy.get_param("~arm/joint_speed"),
                                            j_a=rospy.get_param("~arm/joint_acceleration"))
        rospy.loginfo("EquipmentTask.__init__(): initialized")

    def select_equipment(self, name="Smart_Sensor_Box"):
        """
        Select one of the equipment defined by the corresponding yaml file on the parameter server
        :param name: name of the equipment
        :type name: basestring
        :return: True if equipment was selected
        :rtype: bool
        """
        retValue = False
        for item in self.lst_equipment:
            retValue = item.name == name
            if retValue:
                self.selected_equipment = item
                return retValue
        return retValue

    def generate_goal(self, query_pose):
        """
        The pose may not be given in a frame suitable for stable planing, therefore you may change its reference frame
        and pose accordingly here
        :param query_pose: pose given
        :type query_pose: PoseStamped
        :return: pose expressed in suitable reference frame
        :rtype: PoseStamped
        """
        # rospy.loginfo("EquipmentTask.generate_goal(): query_pose: \n %s", query_pose)
        ret_value = self.moveit.attached_equipment
        if ret_value is None:
            ret_value = self.selected_equipment
        ret_pose = ret_value.get_grasp_pose(query_pose, tf_listener=self.tf_listener)
        rospy.loginfo("EquipmentTask.generate_goal():  goal_pose: \n %s", ret_pose)
        return ret_pose

    def perform(self, stages=None):
        """
        Equipment Handle Task:
            -1. Open Hand
            0. Scan Environment for obstacles
            1. Sense for target pose
            2. Pick Up Equipment (PreGrasp, Grasp)
            3. Update Planning Scene (PostGrasp)
            4. Query Set Pose
            5. Calculate target pose
            6. Move to intermediate pose (INTERMED_POSE)
            7. Move to set pose (TARGET_POSE)
            8. Return Equipment or Open Hand ([Grasp])
            9. Return to Home Pose (HOME)
        :return: -
        :rtype: -
        """
        if stages is None:
            stages = range(9)
        stages = frozenset(stages)
        debug_pose_pub = rospy.Publisher("debug_target_pose", PoseStamped)
        query_pose = None
        arm_frame = "gripper_ur5_base_link"
        # 0. Open hand, in case smart equipment is still stuck
        if -1 in stages:
            rospy.loginfo("STAGE -1: Open Hand")
            rospy.loginfo("EquipmentTask.perform([0]): Equipment handling started - Robot starting at HOME position")
            self.hand_controller.openHand()
            self.moveit.move_to_target(self.home_joint_values, info="HOME")
        # 0. Scan Environment
        if 0 in stages:
            rospy.loginfo("STAGE 0: Scan Environment for obstacles")
            rospy.loginfo("EquipmentTask.perform([0]): Closing hand and scan the environment by given watch pose")
            self.hand_controller.closeHand()
            # Scan env
            for joint_target in self.env_sense_joints_poses:
                self.moveit.move_to_target(joint_target, info="ENV_SCAN")

        if 1 in stages:
            rospy.loginfo("STAGE 1: Sense for target pose")
            self.moveit.move_to_target(self.watch_joint_values, info="Watch Pose")
            # Sense for target_pose
            target_on_floor = sense()  # type: PoseStamped
            # Assume that the base_link doesn't move, so we can save the pose relative to it
            self.tf_listener.waitForTransform(target_frame=arm_frame, source_frame=target_on_floor.header.frame_id,
                                              time=target_on_floor.header.stamp, timeout=rospy.Duration(15))
            self.selected_equipment.place_ps = self.tf_listener.transformPose(target_frame=arm_frame,
                                                                              ps=target_on_floor)
            rospy.loginfo("EquipmentTask.perform([1]): Sensed for target_pose %s" % self.selected_equipment.place_ps)

        # 2. Pick Up Equipment
        if 2 in stages:
            rospy.loginfo("STAGE 2: Pickup Equipment")
            # TODO: Select Equipment
            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["pre_grasp"],
                                                 info="PreGrasp"):
                rospy.loginfo("EquipmentTask.perform([2]): Try to plan pre-grasping again")
            # rospy.logdebug("EquipmentTask.perform(): Opening hand ...")
            self.hand_controller.openHand()
            rospy.sleep(2.01)
            # self.hand_controller.closeHand("scissor")

            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="Grasp"):
                rospy.loginfo("EquipmentTask.perform([2]): Try to plan grasping again")
            # rospy.logdebug("EquipmentTask.perform(): Grasp equipment")
            # Grasp station
            self.hand_controller.closeHand()

        # 3. Update Planning Scene - Attach collision object to end effector
        if 3 in stages:
            rospy.loginfo("STAGE 3: Update scene, Attach object")
            rospy.loginfo("EquipmentTask.perform(3): Attach equipment to end effector")
            # Transform all coordinates relevant in <arm_frame>
            self.tf_listener.waitForTransform(arm_frame, self.selected_equipment.ps.header.frame_id,
                                              rospy.Time(0),
                                              rospy.Duration(4))
            self.selected_equipment.ps = self.tf_listener.transformPose(target_frame=arm_frame,
                                                                        ps=self.selected_equipment.ps)
            self.tf_listener.waitForTransform(arm_frame, self.selected_equipment.place_ps.header.frame_id,
                                              rospy.Time(0),
                                              rospy.Duration(4))
            self.selected_equipment.place_ps = self.tf_listener.transformPose(target_frame=arm_frame,
                                                                              ps=self.selected_equipment.place_ps)

            # Adjust SSB rotation
            aTs = pose_to_array(self.selected_equipment.place_ps.pose)  # in Arm coordinates, ie relative to <arm_frame>
            sTg = pose_to_array(self.selected_equipment.get_grasp_pose(object_pose_stamped=self.selected_equipment.ps,
                                                                       tf_listener=self.tf_listener, save_relation=True,
                                                                       use_relation=False).pose)
            n_aTs = optimize_ssb_z_rotation3(oTs=aTs, sTg=sTg)
            # rospy.logdebug("EquipmentTask.perform(3): Calculating gripper Rotation")
            # rospy.logdebug("EquipmentTask.perform(3): aTs\n%s" % aTs)
            # rospy.logdebug("EquipmentTask.perform(3): sTg\n%s" % sTg)
            # rospy.logdebug("EquipmentTask.perform(3): n_aTs\n%s" % n_aTs)
            self.selected_equipment.place_ps.header.frame_id = arm_frame
            self.selected_equipment.place_ps.pose = array_to_pose(n_aTs)
            self.moveit.attach_equipment(self.selected_equipment)

            # self.selected_equipment.place_ps was set earlier
            gp = copy.deepcopy(self.selected_equipment.place_ps)
            gp.pose = array_to_pose(np.matmul(pose_to_array(self.selected_equipment.place_ps.pose), sTg))
            marker_at_ps(self.selected_equipment.place_ps, gripper_pose=gp)
            query_pose = gp
            # Now we can already check if their will be a solution for our IK
            # rospy.loginfo("EquipmentTask.perform([3]): Testing for IK solution")
            # self.moveit.clear_octomap_on_marker(int_marker)
            # while self.moveit.plan(int_marker.gripper_pose) is None:
            #     rospy.logerror("EquipmentTask.perform([3]): Can't find IK solution for pose \n%s" % int_marker.gripper_pose)
            #     rospy.sleep(2.0)
            # rospy.loginfo("EquipmentTask.perform([3]): Found IK solution for pose \n%s" % int_marker.gripper_pose)
            self.moveit.clear_octomap_via_box_marker()
            if "lift" in self.selected_equipment.pickup_waypoints:
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift")
            self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostGrasp")

        set_successfully = 4 not in stages or 5 not in stages
        intermediate_pose = None
        while not set_successfully:
            target_pose = None
            # 4. Query Goal from User Interface
            if 4 in stages and query_pose is None:
                #     rospy.loginfo("STAGE 4: Ask for target Pose")
                #     int_marker.enable_marker()
                #     query_pose_topic = int_marker.get_pose_topic()
                #     query_pose_subscriber = message_filters.Subscriber(query_pose_topic, PoseStamped)
                #     query_pose_cache = message_filters.Cache(query_pose_subscriber, 5)
                #     while query_pose is None:
                #         query_pose = query_pose_cache.getLast()
                #         self.selected_equipment.place_ps = query_pose
                #         rospy.sleep(0.5)
                #     int_marker.disable_marker()
                #     del query_pose_cache
                #     del query_pose_subscriber
                #     del query_pose_topic
                #
                # if query_pose is None:
                rospy.logwarn("EquipmentTask.perform(): There is no interactive marker for the target pose")
                query_pose = self.selected_equipment.place_ps
            rospy.logdebug("EquipmentTask.perform([4]): Set equipment to pose: \n %s", query_pose)

            # 5. Calculate target pose
            if 5 in stages:
                rospy.loginfo("STAGE 5: Calculate Target Pose")
                rospy.loginfo("EquipmentTask.perform([5]): Set equipment ...")
                # Formulate Planning Problem
                self.moveit.clear_octomap_on_marker(marker_at_ps(self.selected_equipment.place_ps))
                target_pose = query_pose
                # try:
                #     target_pose = self.moveit.attached_equipment.get_grasp_pose(query_pose, self.tf_listener,
                #                                                                 save_relation=False, use_relation=True)
                # except AttributeError:
                #     target_pose = query_pose
                #     rospy.logwarn("EquipmentTask.perform([5]): No attached equipment and therefore no gripper pose")
                # rospy.logdebug("EquipmentTask.perform([5]): Target Pose from Marker is:\n%s", target_pose)
                if target_pose is None:
                    rospy.loginfo("EquipmentTask.perform([5]): No Target Pose")
                    continue

            if 6 in stages:
                rospy.loginfo("STAGE 6: Move to Intermediate Pose")
                # Plan Path using Moveit
                intermediate_pose = copy.deepcopy(target_pose)
                intermediate_pose.pose.position.z += 0.3
                debug_pose_pub.publish(intermediate_pose)
                rospy.loginfo("EquipmentTask.perform([6]): Intermediate Pose published")
                if not self.moveit.move_to_target(intermediate_pose, info="INTERMED_POSE"):
                    rospy.loginfo("EquipmentTask.perform([6]): Intermediate Pose not reached - Please set Pose again")
                    continue

            if 7 in stages:
                rospy.loginfo("STAGE 7: Move to Target Pose")
                debug_pose_pub.publish(target_pose)
                set_successfully = self.moveit.move_to_target(target_pose, info="TARGET_POSE")

        if 8 in stages:
            rospy.loginfo("STAGE 8: Set equipment to Target Pose")
            if self.selected_equipment.hold_on_set != 0.0:
                rospy.sleep(self.selected_equipment.hold_on_set)
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostGrasp")
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift")
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="Grasp")
            rospy.loginfo("EquipmentTask.perform([8]): Release Equipment")
            self.hand_controller.openHand()
            self.moveit.detach_equipment()

        if 9 in stages:
            rospy.loginfo("STAGE 9: Return to home pose")
            # Plan back to home station
            if self.selected_equipment.hold_on_set == 0.0:
                # Now release the station in a manner that it stays on the floor
                # moving eef
                rospy.sleep(2.0)
                self.hand_controller.closeHand(mode="scissor")
                rospy.sleep(2.0)
                release_pose = PoseStamped()
                release_pose.header.frame_id = self.moveit.eef_link
                release_pose.pose = array_to_pose(array_from_xyzrpy([-0.1, 0, -0.075], np.deg2rad([0, -45.0, 0])))
                self.moveit.move_to_target(release_pose, "relative")
                while not self.moveit.move_to_target(release_pose, info="RELEASE_POSE"):
                    rospy.loginfo("EquipmentTask.perform([9]): Release Pose not reached - Trying again")
                self.hand_controller.closeHand()
                if intermediate_pose is None:
                    intermediate_pose = release_pose
                while not self.moveit.move_to_target(intermediate_pose, info="INTERMED_POSE"):
                    rospy.loginfo("EquipmentTask.perform([9]): Intermediate Pose not reached - Trying again")

            self.moveit.move_to_target(self.home_joint_values, info="HOME")
            # self.moveit.remove_equipment(self.selected_equipment.name)

        rospy.loginfo("EquipmentTask.perform(): Finished")

    def start(self):
        """
        Start the equipment handle task
        :return: -
        :rtype: -
        """
        # self.perform([3])
        self.perform([1, 2, 3, 4, 5, 6, 7, 8, 9])
        # self.perform([-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        # self.perform([9])


def marker_at_ps(ps_marker, gripper_pose=None):
    """
    visualize a interactive marker at the given ps
    :param ps_marker:
    :return:
    """
    if gripper_pose is None:
        int_marker = marker.SSBMarker("debug_marker", pose=ps_marker, controls="r")
    else:
        int_marker = marker.SSBGraspedMarker(name="debug_grasped_marker", pose=ps_marker, gripper_pose=gripper_pose,
                                             controls="")
    int_marker.enable_marker()
    return int_marker


if __name__ == '__main__':
    rospy.init_node("EquipmentTask", log_level=rospy.DEBUG)
    obj = EquipmentTask()
    obj.hand_controller.openHand()
    rospy.sleep(1.0)
    obj.hand_controller.closeHand()
    for i in range(0, len(obj.lst_equipment)):
        eq = obj.lst_equipment[i]  # type: SmartEquipment
        if obj.select_equipment(eq.name):
            obj.start()
