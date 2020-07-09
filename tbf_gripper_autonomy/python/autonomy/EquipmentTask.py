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

from object_detector.srv import LocateInCloud, LocateInCloudRequest, LocateInCloudResponse
from std_msgs.msg import Float32


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

        x = (ca * cg - sa * cb * sg) * px + (-ca * sg - sa * cb * cg) * py + sa * sb * pz + qx
        y = (sa * cg + ca * cb * sg) * px + (-sa * sg + ca * cb * cg) * py - ca * sb * pz + qy
        z = sb * sg * px - sb * cg * py + cb * pz + qz

        dx = (-ca * sg - sa * cb * cg) * px + (-ca * cg + sa * cb * sg) * py
        dy = (-sa * sg + ca * cb * cg) * px + (-sa * cg - ca * cb * sg) * py
        dz = sb * cg * px + sb * sg * py

        ddx = (-ca * cg + sa * cb * sg) * px + (ca * sg - sa * cb * cg) * py
        ddy = (-sa * cg - ca * cb * sg) * px + (sa * sg - ca * cb * cg) * py
        ddz = -sb * sg * px + sb * cg * py

        v = x ** 2 + y ** 2 + z ** 2
        dv = 2 * (x * dx + y * dy + z * dz)
        ddv = 2 * (dx ** 2 + x * ddx + dy ** 2 + y * ddy + dz ** 2 + z * ddz)

        f = np.sqrt(v)
        df = dv / (2. * f)
        ddf = ((ddv / 2.) - df ** 2) / f

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
            return _newton(a, b, g + np.pi * 0.3, sph, aps, e)
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

        self.selected_equipment = None
        if len(self.lst_equipment) > 0:
            self.selected_equipment = self.lst_equipment[0]  # type: SmartEquipment
        rospy.loginfo("EquipmentTask.__init__(): Equipment initialized")

        # Static joint values for specific well known poses
        self.backup_joint_values = rospy.get_param("~arm/backup_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.home_joint_values = rospy.get_param("~arm/home_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.watch_joint_values = rospy.get_param("~arm/watch_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.env_sense_joints_poses = rospy.get_param("~arm/env_scan_joint_poses", [])

        # @All parameters were imported@
        super(EquipmentTask, self).__init__(js_t=rospy.get_param("~arm/joint_states_topic", "/ur5/joint_states"),
                                            bu_pos=self.backup_joint_values,
                                            ltcp_s=rospy.get_param("~arm/linear_tcp_speed", 0.1),
                                            ltcp_a=rospy.get_param("~arm/linear_tcp_acceleration", 0.04),
                                            j_s=rospy.get_param("~arm/joint_speed", 5.0),
                                            j_a=rospy.get_param("~arm/joint_acceleration", 1.0))
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
            0. Initialize
            1. Scan environment
            2. Sense for target pose
            3. Grasp SSB
            4. Move SSB
            5. Release SSB
            6. Return to Home Pose (HOME)
        :return: -
        :rtype: -
        """
        if stages is None:
            stages = range(6)
        stages = frozenset(stages)
        debug_pose_pub = rospy.Publisher("debug_target_pose", PoseStamped)
        target_pose = None
        intermediate_pose = None
        arm_frame = "gripper_ur5_base_link"
        if 0 in stages:
            rospy.loginfo("STAGE 0: Initialize")
            rospy.loginfo("EquipmentTask.perform([0]): Equipment handling started - Robot starting at HOME position")
            self.hand_controller.openHand()  # Open hand, in case smart equipment is still stuck
            self.moveit.move_to_target(self.home_joint_values, info="HOME")
        if 1 in stages:
            rospy.loginfo("STAGE 0: Scan Environment for obstacles")
            rospy.loginfo("EquipmentTask.perform([0]): Closing hand and scan the environment by given watch pose")
            self.hand_controller.closeHand()
            # Scan env
            for joint_target in self.env_sense_joints_poses:
                self.moveit.move_to_target(joint_target, info="ENV_SCAN")

        if 2 in stages:
            rospy.loginfo("STAGE 2: Sense for target pose")
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
        if 3 in stages:
            rospy.loginfo("STAGE 3: Pickup Equipment")
            # TODO: Select Equipment
            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["pre_grasp"],
                                                 info="PreGrasp"):
                rospy.loginfo("EquipmentTask.perform([2]): Try to plan pre-grasping again")
            self.hand_controller.openHand()
            rospy.sleep(2.01)
            self.hand_controller.closeHand("scissor")

            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="Grasp"):
                rospy.loginfo("EquipmentTask.perform([3]): Try to plan grasping again")
            # Grasp station
            self.hand_controller.closeHand("basic")

            # Update Planning Scene - Attach collision object to end effector
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
            n_aTs = optimize_ssb_z_rotation(oTs=aTs, sTg=sTg)
            self.selected_equipment.place_ps.header.frame_id = arm_frame
            self.selected_equipment.place_ps.pose = array_to_pose(n_aTs)
            self.moveit.attach_equipment(self.selected_equipment)

            # self.selected_equipment.place_ps was set earlier
            target_pose = copy.deepcopy(self.selected_equipment.place_ps)
            target_pose.pose = array_to_pose(np.matmul(pose_to_array(self.selected_equipment.place_ps.pose), sTg))
            marker_at_ps(self.selected_equipment.place_ps, gripper_pose=target_pose)

            if "lift" in self.selected_equipment.pickup_waypoints:
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift")
            self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostGrasp")

        # 5. Calculate target pose
        if 4 in stages:
            rospy.loginfo("STAGE 4: Move SSB %s" % self.selected_equipment.name)
            # Formulate Planning Problem
            if target_pose is None:
                rospy.loginfo("EquipmentTask.perform([5]): No Target Pose")
                return

            rospy.loginfo("STAGE 4: Move to Intermediate Pose")
            # Plan Path using Moveit
            intermediate_pose = copy.deepcopy(target_pose)
            intermediate_pose.pose.position.z += 0.3
            debug_pose_pub.publish(intermediate_pose)
            rospy.loginfo("EquipmentTask.perform([4]): Intermediate Pose published")
            while not self.moveit.move_to_target(intermediate_pose, info="INTERMED_POSE", endless=False):
                rospy.loginfo("EquipmentTask.perform([4]): Intermediate Pose not reached - Trying again")

            rospy.loginfo("STAGE 4: Move to Target Pose")
            # self.moveit.clear_octomap_on_mesh(ps=self.selected_equipment.place_ps, mesh=self.selected_equipment.mesh_path)
            debug_pose_pub.publish(target_pose)
            self.moveit.move_to_set(target_pose, info="TARGET_POSE")

        if 5 in stages:
            rospy.loginfo("STAGE 8: Release/Hold SSB %s" % self.selected_equipment.name)
            if self.selected_equipment.hold_on_set != 0.0:
                # TODO: Test this path
                rospy.sleep(self.selected_equipment.hold_on_set)
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostGrasp")
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift")
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="Grasp")
            rospy.loginfo("EquipmentTask.perform([8]): Release Equipment")
            self.hand_controller.openHand()
            self.moveit.detach_equipment()
            if self.selected_equipment.hold_on_set == 0.0:
                # Now release the station in a manner that it stays on the floor
                # moving eef
                self.hand_controller.closeHand(mode="scissor")
                hand_frame = "gripper_robotiq_palm_planning"  # = self.moveit.group.get_planning_frame()
                self.tf_listener.waitForTransform(hand_frame, target_pose.header.frame_id, target_pose.header.stamp,
                                                  timeout=rospy.Duration(10))
                release_pose = self.tf_listener.transformPose(hand_frame, target_pose)  # type: PoseStamped
                release_pose.pose.position.x = release_pose.pose.position.x - 0.15
                from scipy.spatial.transform import Rotation as R
                r_is = R.from_quat([release_pose.pose.orientation.x, release_pose.pose.orientation.y,
                                    release_pose.pose.orientation.z, release_pose.pose.orientation.w])
                r_relative = R.from_euler('z', -20, degrees=True)
                r_soll = r_is
                while not self.moveit.move_to_target(release_pose, info="RELEASE_POSE", endless=True):
                    release_pose.pose.position.x = release_pose.pose.position.x + 0.05
                    r_soll = r_soll * r_relative
                    q = r_soll.as_quat()
                    release_pose.pose.orientation.x = q[0]
                    release_pose.pose.orientation.y = q[1]
                    release_pose.pose.orientation.z = q[2]
                    release_pose.pose.orientation.w = q[3]
                    rospy.loginfo("EquipmentTask.perform([9]): Release Pose not reached - Trying again")
                self.hand_controller.closeHand()
                while not self.moveit.move_to_target(intermediate_pose,
                                                     info="INTERMED_POSE") and intermediate_pose is not None:
                    rospy.loginfo("EquipmentTask.perform([9]): Intermediate Pose not reached - Trying again")

        if 6 in stages:
            rospy.loginfo("STAGE 6: Return to home pose")
            # Plan back to home station

            # if intermediate_pose is not None:
            #     self.moveit.move_to_target(intermediate_pose, info="INTERMED_POSE", endless=False)
            self.moveit.move_to_target(self.watch_joint_values, info="Watch Pose")
            # self.moveit.move_to_target(self.home_joint_values, info="HOME")
            # self.moveit.remove_equipment(self.selected_equipment.name)

        rospy.loginfo("EquipmentTask.perform(): Finished")

    def start(self):
        """
        Start the equipment handle task
        :return: -
        :rtype: -
        """
        # self.perform([2, 3, 4, 5, 6])
        self.perform([0, 1, 2, 3, 4, 5, 6])

    def check_set_equipment_pose(self):
        """
        Move the camera to observe the previuous set smart equipment and the query the object detector via service call
        :return:
        """
        self.hand_controller.closeHand(mode="basic")
        ssb_pose = self.selected_equipment.place_ps
        self.moveit.move_to_target(self.watch_joint_values, info="Watch Pose")
        arm_frame = "gripper_ur5_base_link"
        self.tf_listener.waitForTransform(arm_frame, ssb_pose.header.frame_id, ssb_pose.header.stamp,
                                          timeout=rospy.Duration(10.0))
        watch_pose = self.tf_listener.transformPose(arm_frame, ssb_pose)
        watch_pose.pose.position.z = 0.5
        from scipy.spatial.transform import Rotation as R
        r_zrot = R.from_euler('z', -180, degrees=True)
        r_yrot = R.from_euler('y', -180, degrees=True)
        r_soll = r_yrot * r_zrot
        q = r_soll.as_quat()
        watch_pose.pose.orientation.x = q[0]
        watch_pose.pose.orientation.y = q[1]
        watch_pose.pose.orientation.z = q[2]
        watch_pose.pose.orientation.w = q[3]
        detected_ssb_pose = None
        while detected_ssb_pose is None:
            rospy.loginfo("EquipmentTask.check_set_equipment_pose(): No SSB detected")
            self.moveit.move_to_target(watch_pose, info="DETECT_POSE", endless=True)
            detected_ssb_pose = object_detection()
            watch_pose.pose.position.z += 0.1
        diff = self.compute_ssb_delta(self.selected_equipment.place_ps, detected_ssb_pose)
        rospy.loginfo("EquipmentTask.check_set_equipment_pose() Difference is %s" % diff)
        rospy.Publisher("/ssb_delta", Float32, queue_size=10).publish(diff)

    def compute_ssb_delta(self, ps_0, ps_1):
        """
        compute the euclidean distance between two poses
        :param ps_0: first pose
        :type ps_0: PoseStamped
        :param ps_1: second pose
        :type ps_1: PoseStamped
        :return: difference
        :rtype: float
        """
        self.tf_listener.waitForTransform(ps_0.header.frame_id, ps_1.header.frame_id, ps_1.header.stamp,
                                          timeout=rospy.Duration(10.0))
        ps1 = self.tf_listener.transformPose(ps_0.header.frame_id, ps_1)
        T0 = pose_to_array(ps_0)
        T1 = pose_to_array(ps1)
        return np.linalg.norm(T1[:3, 3] - T0[:3, 3])


def object_detection():
    """
    Connect to <~service_name> and request a <LocateInCloud> for the next point cloud on <~cloud_topic>
    :return: Response from service call
    :rtype: LocateInCloudResponse
    """
    from sensor_msgs.msg import PointCloud2
    pcl_msg = rospy.wait_for_message(rospy.get_param("~cloud_topic", default="/gripper_d435/depth_registered/points"),
                                     PointCloud2, rospy.Duration(10))
    if pcl_msg is None:
        return None
    service_name = rospy.get_param("~service_name", default='locate_ssb_in_cloud')
    rospy.wait_for_service(service_name)
    locate_service = rospy.ServiceProxy(service_name, LocateInCloud)
    if locate_service is None:
        return None
    request = LocateInCloudRequest()
    request.cloud_msg = pcl_msg
    return locate_service(request)


def marker_at_ps(ps_marker, gripper_pose=None):
    """
    visualize a interactive marker at the given ps
    :param gripper_pose: if given mark an arrow there
    :param ps_marker: pose of the marker
    :return: interactive marker
    """
    if gripper_pose is None:
        int_marker = marker.SSBMarker("debug_marker", pose=ps_marker, controls="")
    else:
        int_marker = marker.SSBGraspedMarker(name="debug_grasped_marker", pose=ps_marker, gripper_pose=gripper_pose,
                                             controls="")
    int_marker.enable_marker()
    return int_marker


if __name__ == '__main__':
    rospy.init_node("EquipmentTask", log_level=rospy.INFO)
    obj = EquipmentTask()
    obj.hand_controller.openHand()
    rospy.sleep(rospy.Duration(1))
    obj.hand_controller.closeHand()
    for i in range(0, len(obj.lst_equipment)):
        eq = obj.lst_equipment[i]  # type: SmartEquipment
        if obj.select_equipment(eq.name):
            obj.start()
            # Now we can test if we see the SSB where we think it is
            obj.check_set_equipment_pose()
    # obj.moveit.clear_octomap_on_mesh(obj.selected_equipment.place_ps, obj.selected_equipment.mesh_path)
    # rospy.spin()
