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
import tf2_ros
import message_filters
import numpy as np

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
import std_msgs.msg

from autonomy.Task import GraspTask
from autonomy.MoveitInterface import MoveitInterface
from tbf_gripper_tools.SmartEquipment import SmartEquipment
from tubaf_tools import array_to_pose, pose_to_array
import tbf_gripper_rviz.ssb_marker as marker
from tubaf_tools.confirm_service import wait_for_confirmation
from tf import TransformListener

from object_detector.srv import LocateInCloud, LocateInCloudRequest, LocateInCloudResponse
from std_msgs.msg import Float32

from evaluation.EvaluateEquipmentTask import EquipmentTask as Evaluation


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
    from tbf_gripper_perception.srv import IdentifyFloor, IdentifyFloorRequest, IdentifyFloorResponse
    from std_msgs.msg import Header
    from message_filters import Subscriber, Cache
    from object_recognition_msgs.msg import TableArray
    from visualization_msgs.msg import MarkerArray

    # Identify the floor
    floor_identify_service_name = rospy.get_param("~floor_identify_service", "/ork/identify_floor_plane")
    rospy.loginfo("[sense()] floor_identify_service_name: %s" % floor_identify_service_name)
    rospy.wait_for_service(floor_identify_service_name)
    rospy.loginfo("[sense()] %s available" % floor_identify_service_name)
    floor_identify_service = rospy.ServiceProxy(floor_identify_service_name, IdentifyFloor)
    identify_floor_req = IdentifyFloorRequest()
    identify_floor_res = IdentifyFloorResponse()
    while not rospy.is_shutdown():
        planes = rospy.wait_for_message(rospy.get_param('~tables_topic', "/ork/table_array"), TableArray)
        identify_floor_req.planes = planes
        identify_floor_res = floor_identify_service(identify_floor_req)
        if identify_floor_res.success:
            break
        else:
            rospy.logwarn("[sense()] Identify floor plane failed")
            rospy.sleep(2.0)

    # As client
    # use: PcaPoseGenerator, MinimalDensityEstimatePoseGenerator, DelaunayPoseGenerator
    pose_generation_service_name = rospy.get_param("~sense_service_name", "DelaunayPoseGenerator_service")
    rospy.loginfo("[sense()] pose_generation_service_name: %s" % pose_generation_service_name)
    rospy.wait_for_service(pose_generation_service_name)
    rospy.loginfo("[sense()] %s available" % pose_generation_service_name)
    pose_generation_service = rospy.ServiceProxy(pose_generation_service_name, GenerateSetPose)

    request = GenerateSetPoseRequest()
    request.header = Header()
    request.header.stamp = rospy.Time.now()
    request.print_evaluation = False
    request.policy = "hl"
    request.floor = identify_floor_res.floor

    _obstacle_topic = rospy.get_param("~obstacle_topic", "/ork/tabletop/clusters")
    _obstacle_cache = Cache(Subscriber(_obstacle_topic, MarkerArray), 1, allow_headerless=True)

    ps = None

    while ps is None:
        try:
            if _obstacle_cache.getLast() is None:
                rospy.sleep(1.0)
                continue
            request.obstacles = _obstacle_cache.getLast()
            # rospy.logdebug("[sense()] Request:\n%s" % request)
            reply = pose_generation_service(request)
            rospy.loginfo("[sense()] %s suggests %s" % (pose_generation_service_name, reply))

            ps = reply.set_pose

        except rospy.ServiceException as e:
            rospy.logerr("[main] Service %s call failed\n%s" % (pose_generation_service_name, e.message))
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

    def __init__(self, evaluation=None):
        """
        Default constructor, start ROS, hand_model and demo_monitoring
        """
        if evaluation is None:
            self.evaluation = False
        else:
            self.evaluation = evaluation  # type: Evaluation
        self.tf_listener = tf.TransformListener(rospy.Duration.from_sec(15.0))
        self.tfBuffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tfBuffer)
        # Init Moveit
        self.moveit = MoveitInterface("~moveit", self.tf_listener, evaluation=self.evaluation)  # type: MoveitInterface
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
        ret_value = False
        for item in self.lst_equipment:
            ret_value = item.name == name
            if ret_value:
                self.selected_equipment = item
                return ret_value
        return ret_value

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

    def generate_path_constraints(self):
        """
        Generate path constraints for the ssb handling
        :return: constraints for MoveIt!
        :rtype: Constraints
        """
        planning_frame = self.moveit.group.get_pose_reference_frame()
        trans = self.tfBuffer.lookup_transform(self.moveit.eef_link, planning_frame, rospy.Time(0))

        c_orientation = OrientationConstraint()
        c_orientation.header = trans.header
        c_orientation.link_name = trans.child_frame_id
        c_orientation.orientation = trans.transform.rotation
        c_orientation.absolute_z_axis_tolerance = 2 * np.pi

        constraints = Constraints()
        constraints.name = "SSB_handling_constraint"
        constraints.orientation_constraints.append(c_orientation)

    def sense(self, arm_frame):
        """
        Sense for a pose to set the equipment
        :return:
        """
        approved = False
        candidate_pose = None
        while not approved:
            target_on_floor = sense()  # type: PoseStamped
            # Assume that the base_link doesn't move, so we can save the pose relative to it
            self.tf_listener.waitForTransform(target_frame=arm_frame, source_frame=target_on_floor.header.frame_id,
                                              time=target_on_floor.header.stamp, timeout=rospy.Duration(15))
            candidate_pose = self.tf_listener.transformPose(target_frame=arm_frame, ps=target_on_floor)
            if self.evaluation:
                self.evaluation.pause()
            marker_at_ps(candidate_pose)
            rospy.loginfo("EquipmentTask.sense(): Please confirm the pose using the interactive marker on topic: "
                          "'/confirm_set_pose/markers/update'")
            approved = wait_for_confirmation(service_ns="~confirm_set_pose", timeout=20)
            if self.evaluation:
                self.evaluation.resume()
        return candidate_pose

    def perform(self, stages=None):
        """
        Equipment Handle Task:
            0. Initialize
            1. Scan environment
            2. Sense for target pose
            3. Pick SSB
            4. Place SSB
            5. Release SSB
            6. Search SSB
            7. Return to Home Pose (HOME)
        :return: -
        :rtype: -
        """
        if stages is None:
            stages = range(6)
        stages = frozenset(stages)
        debug_pose_pub = rospy.Publisher("debug_target_pose", PoseStamped)
        target_pose = None
        intermediate_pose = None
        constraints = None
        arm_frame = "gripper_ur5_base_link"
        if 0 in stages:
            rospy.loginfo("STAGE 0: Initialize")
            rospy.logdebug("EquipmentTask.perform([0]): Equipment handling started - Robot starting at HOME position")
            self.hand_controller.openHand()  # Open hand, in case smart equipment is still stuck
            self.moveit.move_to_target(self.home_joint_values, info="HOME")
        if 1 in stages:
            if self.evaluation:
                self.evaluation.t_start = rospy.Time.now()
            rospy.loginfo("STAGE 0: Scan Environment for obstacles")
            rospy.logdebug("EquipmentTask.perform([0]): Closing hand and scan the environment by given watch pose")
            self.hand_controller.closeHand(continue_image_service=True)
            # Scan env
            i = 0
            for joint_target in self.env_sense_joints_poses:
                title = "Scan"+str(i)
                self.moveit.move_to_target(joint_target, info=title, blind=True)
                if self.evaluation:
                    self.evaluation.store_img(title)
                i += 1

        if 2 in stages:
            rospy.loginfo("STAGE 2: Sense for target pose")
            self.moveit.move_to_target(self.watch_joint_values, info="Sense", blind=True)
            if self.evaluation:
                self.evaluation.store_img("Sense")
            # Sense for target_pose
            self.selected_equipment.place_ps = self.sense(arm_frame)
            rospy.logdebug("EquipmentTask.perform([1]): Sensed for target_pose %s" % self.selected_equipment.place_ps)

        # 2. Pick Up Equipment
        if 3 in stages:
            rospy.loginfo("STAGE 3: Pickup Equipment")
            self.hand_controller._set_image_and_info_service(False)
            # TODO: Select Equipment
            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["pre_grasp"], info="PrePick",
                                                 blind=True):
                rospy.logdebug("EquipmentTask.perform([2]): Try to plan pre-grasping again")
            if self.evaluation:
                self.evaluation.store_img("PrePick")
            self.hand_controller.openHand()
            self.hand_controller.closeHand("scissor", continue_image_service=False)

            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="PickClose",
                                                 blind=True):
                rospy.logdebug("EquipmentTask.perform([3]): Try to plan grasping again")
            # Grasp station
            self.hand_controller.closeHand("basic", continue_image_service=False)
            if self.evaluation:
                self.evaluation.store_img("PickClose")
            # Update Planning Scene - Attach collision object to end effector
            rospy.loginfo("STAGE 3: Update scene, Attach object")
            rospy.logdebug("EquipmentTask.perform(3): Attach equipment to end effector")
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
            # TODO: Fix Internal error. <Mesh filter handle 85 not found> from move_group node, STILL relevant?
            self.moveit.attach_equipment(self.selected_equipment)

            # self.selected_equipment.place_ps was set earlier
            target_pose = copy.deepcopy(self.selected_equipment.place_ps)  # target pose is in arm_frame
            target_pose.pose = array_to_pose(np.matmul(pose_to_array(self.selected_equipment.place_ps.pose), sTg))
            marker_at_ps(self.selected_equipment.place_ps, gripper_pose=target_pose)

            if self.evaluation:
                self.evaluation.grasp_relation = array_to_pose(sTg)
                self.evaluation.estimated_set_pose = self.selected_equipment.place_ps

            constraints = self.generate_path_constraints()

            if "lift" in self.selected_equipment.pickup_waypoints:
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift",
                                           constraints=constraints)
            self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostPick",
                                       constraints=constraints)

        # 5. Calculate target pose
        if 4 in stages:
            rospy.loginfo("STAGE 4: Place SSB %s" % self.selected_equipment.name)
            # Formulate Planning Problem
            if target_pose is None:
                rospy.logdebug("EquipmentTask.perform([5]): No Target Pose")
                return

            rospy.loginfo("STAGE 4: Move to Intermediate Pose")
            # Plan Path using Moveit
            intermediate_pose = copy.deepcopy(target_pose)
            intermediate_pose.pose.position.z += 0.3
            if self.evaluation:
                self.evaluation.intermediate_set_pose = intermediate_pose
            debug_pose_pub.publish(intermediate_pose)
            rospy.logdebug("EquipmentTask.perform([4]): Intermediate Pose published")
            while not self.moveit.move_to_target(intermediate_pose, info="PlaceIntermediate", endless=False,
                                                 constraints=constraints):
                rospy.logdebug("EquipmentTask.perform([4]): Intermediate Pose not reached - Trying again")

            rospy.loginfo("STAGE 4: Move to Target Pose")
            debug_pose_pub.publish(target_pose)
            self.moveit.move_to_set(target_pose, info="PlaceSet",  constraints=constraints)
            if self.evaluation:
                self.evaluation.store_img("PlaceSet")

        if 5 in stages:
            rospy.loginfo("STAGE 5: Release/Hold SSB %s" % self.selected_equipment.name)
            if self.selected_equipment.hold_on_set != 0.0:
                # TODO: Test this path
                rospy.sleep(self.selected_equipment.hold_on_set)
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostGrasp")
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift")
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="Grasp")
            rospy.loginfo("EquipmentTask.perform([5]): Release Equipment")
            self.hand_controller.openHand()
            self.moveit.detach_equipment()
            if self.selected_equipment.hold_on_set == 0.0:
                # Now release the station in a manner that it stays on the floor
                # moving eef
                self.hand_controller.closeHand(mode="scissor", continue_image_service=False)

                hand_frame = "gripper_robotiq_palm_planning"  # = self.moveit.group.get_planning_frame()
                self.tf_listener.waitForTransform(hand_frame, target_pose.header.frame_id, rospy.Time(0),
                                                  rospy.Duration(4))
                release_pose = self.tf_listener.transformPose(target_frame=hand_frame, ps=target_pose)
                # release_pose = self.tfBuffer.transform(target_pose, target_frame=hand_frame,
                #                                        timeout=rospy.Duration(10))  # type: PoseStamped
                rospy.logdebug("EquipmentTask.perform([5]): Release pose was: \n %s" % release_pose)
                release_pose.pose.position.x = release_pose.pose.position.z - 0.15
                from scipy.spatial.transform import Rotation as R
                r_is = R.from_quat([release_pose.pose.orientation.x, release_pose.pose.orientation.y,
                                    release_pose.pose.orientation.z, release_pose.pose.orientation.w])
                r_relative = R.from_euler('z', -20, degrees=True)
                r_soll = r_is
                rospy.logdebug("EquipmentTask.perform([5]): Release pose is: \n %s" % release_pose)
                debug_pose_pub.publish(release_pose)
                i = 0
                while not self.moveit.move_to_target(release_pose, info="PlaceRelease"+str(i), endless=False, blind=True):
                    i += 1
                    release_pose.pose.position.x = release_pose.pose.position.x + 0.01
                    r_soll = r_soll * r_relative
                    q = r_soll.as_quat()
                    release_pose.pose.orientation.x = q[0]
                    release_pose.pose.orientation.y = q[1]
                    release_pose.pose.orientation.z = q[2]
                    release_pose.pose.orientation.w = q[3]
                    rospy.logdebug(
                        "EquipmentTask.perform([5]): Release Pose not reached - Trying again with \n %s" %
                        release_pose.pose)
                    debug_pose_pub.publish(release_pose)
                    if self.evaluation:
                        self.evaluation.store_img("PlaceRelease"+str(i))
                self.hand_controller.closeHand(continue_image_service=False)

        if 6 in stages:
            rospy.loginfo("STAGE 7: Search for SSB")
            self.moveit.move_to_target(self.watch_joint_values, info="SenseAfterwards")
            if self.evaluation:
                self.evaluation.store_img("SenseAfterwards")
                self.evaluation.t_in_s = self.evaluation.calc_time()
            obj.check_set_equipment_pose()

        if 7 in stages:
            rospy.loginfo("STAGE 7: Return to watch pose")
            # Plan back to home station
            self.moveit.move_to_target(self.home_joint_values, info="HOME")
            # self.moveit.remove_equipment(self.selected_equipment.name)

        rospy.loginfo("EquipmentTask.perform(): Finished")

    def start(self):
        """
        Start the equipment handle task
        :return: -
        :rtype: -
        """
        self.perform([0, 1, 2, 3, 4, 5, 6])
        # self.perform([2])

    def check_set_equipment_pose(self):
        """
        Move the camera to observe the previous set smart equipment and the query the object detector via service call
        :return:
        """
        from sensor_msgs.msg import PointCloud2
        pcl_pub = rospy.Publisher("/result/pointcloud", PointCloud2, queue_size=10)
        diff_pub = rospy.Publisher("/result/ssb_delta", Float32, queue_size=10)
        self.hand_controller.closeHand(mode="basic")
        ssb_pose = self.selected_equipment.place_ps
        # self.moveit.move_to_target(self.watch_joint_values, info="Watch Pose")
        arm_frame = "gripper_ur5_base_link"
        # camera_frame = rospy.get_param("camera_frame", default="rs_gripper_d435_color_optical_frame")
        # self.tf_listener.waitForTransform(arm_frame, ssb_pose.header.frame_id, ssb_pose.header.stamp,
        #                                   timeout=rospy.Duration(10.0))
        self.tf_listener.waitForTransform(arm_frame, ssb_pose.header.frame_id, rospy.Time(0),
                                          rospy.Duration(4))
        watch_pose = self.tf_listener.transformPose(target_frame=arm_frame, ps=ssb_pose)
        if self.evaluation:
            self.evaluation.estimated_set_pose = self.tf_listener.transformPose(target_frame=arm_frame, ps=ssb_pose)
        watch_pose.pose.position.x = watch_pose.pose.position.x - 0.14
        watch_pose.pose.position.z = 0.0

        from scipy.spatial.transform import Rotation as R
        r_yrot = R.from_euler('y', 90, degrees=True)
        r_xrot = R.from_euler('x', -90, degrees=True)
        r_soll = r_yrot * r_xrot
        q = r_soll.as_quat()
        # q = [0, 0, 0, 1]  # no rotation matches rotation from arm_frame
        watch_pose.pose.orientation.x = q[0]
        watch_pose.pose.orientation.y = q[1]
        watch_pose.pose.orientation.z = q[2]
        watch_pose.pose.orientation.w = q[3]
        detected_ssb_pose = None
        i = 0
        while detected_ssb_pose is None:
            rospy.loginfo("EquipmentTask.check_set_equipment_pose(): No SSB detected")
            title = "Search"+str(i)
            self.moveit.move_to_target(watch_pose, info=title, endless=True, blind=True)
            detected_ssb_pose, score = object_detection()
            watch_pose.pose.position.z += 0.1
            if self.evaluation:
                self.evaluation.store_img(title)
            i += 1

        if self.evaluation:
            self.evaluation.pause()
            self.evaluation.sensed_pose_confidence = score
            try:
                self.tf_listener.waitForTransform(target_frame=arm_frame, source_frame=detected_ssb_pose.header.frame_id,
                                                  time=detected_ssb_pose.header.stamp, timeout=rospy.Duration())
                self.evaluation.sensed_set_pose = self.tf_listener.transformPose(target_frame=arm_frame, ps=detected_ssb_pose)
            except tf2_ros.tf2.ExtrapolationException as ee:
                rospy.logerr("EquipmentTask.check_set_equipment_pose(): %s" % ee.message)
                self.evaluation.sensed_set_pose = detected_ssb_pose
            self.evaluation.resume()

        diff = self.compute_ssb_delta(detected_ssb_pose)
        rospy.loginfo("EquipmentTask.check_set_equipment_pose() Difference is %s" % diff)

        pcl_pub.publish(rospy.wait_for_message("/gripper_d435/depth/color/points", PointCloud2))
        diff_pub.publish(diff)

    def compute_ssb_delta(self, ps_1):
        """
        compute the euclidean distance between pose and self.selected_equipment.place_ps
        :param ps_1: second pose (/result/observed_pose)
        :type ps_1: PoseStamped
        :return: difference
        :rtype: float
        """
        ps_0 = self.selected_equipment.place_ps
        pub0 = rospy.Publisher("/result/placed_pose", PoseStamped, queue_size=10)
        pub1 = rospy.Publisher("/result/observed_pose", PoseStamped, queue_size=10)
        try:
            ps_1.header.stamp.secs = 0
            ps_1.header.stamp.nsecs = 0
            ps_1.header.seq = 0
            rospy.logdebug("[EquipmentTask.py::compute_ssb_delta()] Placed Pose:\n%s" % ps_0)
            rospy.logdebug("[EquipmentTask.py::compute_ssb_delta()] Observed Pose:\n%s" % ps_1)
            self.tf_listener.waitForTransform(ps_0.header.frame_id, ps_1.header.frame_id, rospy.Time(0),
                                              rospy.Duration.from_sec(15.0))
            ps1 = self.tf_listener.transformPose(ps_0.header.frame_id, ps_1)
            # ps1 = self.tfBuffer.transform(ps_1, ps_0.header.frame_id)
        except Exception as ex:
            rospy.logwarn("[EquipmentTask.py::compute_ssb_delta()] Couldn't transform \n%s" % ex.message)
            pub0.publish(ps_0)
            pub1.publish(ps_1)
            return np.NaN
        T0 = pose_to_array(ps_0.pose)
        T1 = pose_to_array(ps1.pose)

        pub0.publish(ps_0)
        pub1.publish(ps1)

        return np.linalg.norm(T1[:3, 3] - T0[:3, 3])


def object_detection():
    """
    Connect to <~service_name> and request a <LocateInCloud> for the next point cloud on <~cloud_topic>
    :return: Response from service call
    :rtype: PoseStamped, float
    """
    from sensor_msgs.msg import PointCloud2
    pcl_msg = rospy.wait_for_message(rospy.get_param("~cloud_topic", default="/gripper_d435/depth_registered/points"),
                                     PointCloud2, 10)
    if pcl_msg is None:
        return None, -2.0
    service_name = rospy.get_param("~ssb_detection_service_name", default='locate_ssb_in_cloud')
    rospy.wait_for_service(service_name)
    locate_service = rospy.ServiceProxy(service_name, LocateInCloud)
    if locate_service is None:
        return None, -1.0
    request = LocateInCloudRequest()
    request.cloud_msg = pcl_msg
    response = locate_service(request)  # type: LocateInCloudResponse
    rospy.loginfo("[object_detection] response :\n%s" % response)
    if response is None or response.detection_score == 0.0:
        return None, response.detection_score
    return response.object_pose, response.detection_score


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
    rospy.init_node("EquipmentTask", log_level=rospy.DEBUG)
    obj = EquipmentTask(evaluation=Evaluation())

    obj.hand_controller.openHand()
    rospy.sleep(rospy.Duration(1))
    obj.hand_controller.closeHand(continue_image_service=False)
    for i in range(0, len(obj.lst_equipment)):
        eq = obj.lst_equipment[i]  # type: SmartEquipment
        if obj.select_equipment(eq.name):
            try:
                obj.start()
            except Exception as ex:
                rospy.logerr(ex.message)
            finally:
                if obj.evaluation:
                    n = eq.name.split(" ")
                    secs = rospy.Time.now().secs
                    obj.evaluation.save_as_bag("~/bags/EquipmentTask/"+n+str(secs)[6:]+".bag")
            rospy.loginfo("### Finished %s ###" % eq.name)
    rospy.spin()
