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
# Author: grehl
import copy

import rospy
import tf
import message_filters

from geometry_msgs.msg import PoseStamped
import std_msgs.msg

from autonomy.Task import GraspTask
from autonomy.MoveitInterface import MoveitInterface
from tbf_gripper_tools.SmartEquipment import SmartEquipment
import tbf_gripper_rviz.ssb_marker as marker


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


def sense():
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
    service_name = rospy.get_param("~sense_service_name", "DelaunayPoseGenerator")
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

    pose_pub = rospy.Publisher("set_ssb_pose", PoseStamped, queue_size=10)
    ps = None

    while ps is None:
        try:
            request.floor = _floor_cache.getLast()
            request.obstacles = _obstacle_cache.getLast()
            if request.floor is None or request.obstacles is None:
                rospy.sleep(1.0)
                continue
            rospy.logdebug("[main] Request:\n%s" % request)
            reply = service(request)
            rospy.loginfo("[main] %s says %s" % (service_name, reply))

            pose_pub.publish(reply.set_pose)
            ps = reply.set_pose

        except rospy.ServiceException as e:
            rospy.logerr("[main] Service %s call failed\n%s" % (service_name, e.message))
            ps = None
    return ps


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

    def perform(self, stages=range(9)):
        """
        Equipment Handle Task:
            0. Open Hand
            1. Scan Environment (HOME, Watch Pose)
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
        stages = frozenset(stages)
        query_pose = None
        # 0. Open hand, in case smart equipment is still stuck
        if 0 in stages:
            rospy.loginfo("STAGE 0: Open Hand")
            rospy.loginfo("EquipmentTask.perform([0]): Equipment handling started - Robot starting at HOME position")
            self.hand_controller.openHand()
            self.moveit.move_to_target(self.home_joint_values, info="HOME")
        # 1. Scan Environment
        if 1 in stages:
            rospy.loginfo("STAGE 1: Scan Env")
            rospy.loginfo("EquipmentTask.perform([1]): Closing hand and scan the environment by given watch pose")
            self.hand_controller.closeHand()
            self.moveit.move_to_target(self.watch_joint_values, info="Watch Pose")
            # Sense for target_pose
            target_on_floor = sense()
            # sense_pose_subscriber = message_filters.Subscriber(rospy.get_param("~sensed_pose_topic", "/ork/floor_pose"), PoseStamped)
            # sense_pose_cache = message_filters.Cache(sense_pose_subscriber, 5)
            # rospy.sleep(5.0)
            # query_pose = sense_pose_cache.getLast()
            # del sense_pose_cache
            # del sense_pose_subscriber
            query_pose = target_on_floor
            rospy.loginfo("EquipmentTask.perform([1]): Sensed for target_pose %s" % query_pose)
            self.selected_equipment.place_ps = query_pose


        # 2. Pick Up Equipment
        if 2 in stages:
            rospy.loginfo("STAGE 2: Pickup Equipment")
            # TODO: Select Equipment
            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["pre_grasp"], info="PreGrasp"):
                rospy.loginfo("EquipmentTask.perform([2]): Try to plan pre-grasping again")
            # rospy.logdebug("EquipmentTask.perform(): Opening hand ...")
            self.hand_controller.openHand()

            while not self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["grasp"], info="Grasp"):
                rospy.loginfo("EquipmentTask.perform([2]): Try to plan grasping again")
            # rospy.logdebug("EquipmentTask.perform(): Grasp equipment")
            # Grasp station
            self.hand_controller.closeHand()

        debug_pose_pub = rospy.Publisher("debug_target_pose", PoseStamped)
        # 3. Update Planning Scene - Attach collision object to end effector
        if 3 in stages:
            rospy.loginfo("STAGE 3: Update scene, Attach object")
            rospy.loginfo("EquipmentTask.perform(): Attach equipment to end effector")
            self.moveit.attach_equipment(self.selected_equipment)
            self.selected_equipment.calculate_grasp_offset(attached_frame="gripper_robotiq_palm_planning",
                                                           tf_listener=self.tf_listener)  # we use the source frame of the mesh here
            if "lift" in self.selected_equipment.pickup_waypoints:
                self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["lift"], info="Lift")
            self.moveit.move_to_target(self.selected_equipment.pickup_waypoints["post_grasp"], info="PostGrasp")

        set_successfully = False
        int_marker = marker.SSBMarker.from_SmartEquipment(self.selected_equipment)
        intermediate_pose = None
        while not set_successfully:
            target_pose = None
            # 4. Query Goal from User Interface
            if 4 in stages and query_pose is None:
                rospy.loginfo("STAGE 4: Ask for target Pose")
                int_marker.enable_marker()
                query_pose_topic = int_marker.get_pose_topic()
                query_pose_subscriber = message_filters.Subscriber(query_pose_topic, PoseStamped)
                query_pose_cache = message_filters.Cache(query_pose_subscriber, 5)
                while query_pose is None:
                    query_pose = query_pose_cache.getLast()
                    self.selected_equipment.place_ps = query_pose
                    rospy.sleep(0.5)
                int_marker.disable_marker()
                del query_pose_cache
                del query_pose_subscriber
                del query_pose_topic

            if query_pose is None:
                rospy.logwarn("EquipmentTask.perform(): There is no interactive marker for the target pose")
                query_pose = self.selected_equipment.place_ps
            rospy.logdebug("EquipmentTask.perform([4]): Set equipment to pose: \n %s", query_pose)

            # 5. Calculate target pose
            if 5 in stages:
                rospy.loginfo("STAGE 5: Calculate Target Pose")
                rospy.loginfo("EquipmentTask.perform([5]): Set equipment ...")
                # Formulate Planning Problem
                if int_marker is not None:
                    self.moveit.clear_octomap_on_marker(int_marker)
                try:
                    target_pose = self.moveit.attached_equipment.get_grasp_pose(query_pose)
                except AttributeError:
                    target_pose = query_pose
                    rospy.logwarn("EquipmentTask.perform([5]): No attached equipment and therefore no gripper pose")
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
        self.perform([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])


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
