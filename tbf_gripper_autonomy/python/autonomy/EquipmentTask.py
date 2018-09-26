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

import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import numpy as np
import message_filters

from autonomy.Task import GraspTask
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import JointState
import std_msgs.msg

from tbf_gripper_tools.Equipment import Equipment
from tubaf_tools import parse_to_os_path, fill_message
from tubaf_tools.confirm_service import wait_for_confirmation
import tbf_gripper_rviz.ssb_marker as marker


def continue_by_console(text=None):
    """
    Prompt the console with a generic Y/N question and return the result
    :param text: optional quesion
    :type text: String
    :return: result
    :rtype: Bool
    """
    if text is None:
        print("Continue the program (Y/n)?: ")
    else:
        print(text+" (Y/n): ")
    inp = raw_input()
    if inp == "Y":
        return True
    elif inp == "n":
        return False
    else:
        return continue_by_console(text)


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


class MoveitInterface(object):
    """
    Interface to the move_group wrapper
    """
    def __init__(self, param_group_name="~moveit"):
        """
        Default constructor - Parameters are given via yaml file loaded onto the parameter server
        """
        self.parameter = rospy.get_param(param_group_name, dict())
        # Initialize MoveIt!
        # https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/scripts/move_group_python_interface_tutorial.py
        moveit_commander.roscpp_initialize(sys.argv)
        # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("MoveitInterface.__init__(): Planning frame is %s", self.robot.get_planning_frame())
        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding
        #  the robot.
        self.scene = moveit_commander.PlanningSceneInterface()
        # Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints. In this
        # case the group is the joints in the arm.  This interface can be used to plan and execute motions on the
        # arm.
        self.group = moveit_commander.MoveGroupCommander(self.parameter["group_name"])
        self.group.set_planner_id(self.parameter["planner_id"])
        self.group.set_planning_time(self.parameter["planner_time"])
        self.group.set_num_planning_attempts(self.parameter["planner_attempts"])
        self.group.set_goal_tolerance(self.parameter["goal_tolerance"])
        self.group.set_goal_position_tolerance(self.parameter["goal_position_tolerance"])
        self.group.set_goal_orientation_tolerance(self.parameter["goal_orientation_tolerance"])
        self.group.set_goal_joint_tolerance(self.parameter["goal_joint_tolerance"])
        self.group.allow_replanning(self.parameter["allow_replanning"])
        self.group.allow_looking(self.parameter["allow_looking"])
        # [minX, minY, minZ, maxX, maxY, maxZ]
        self.group.set_pose_reference_frame(reference_frame=self.parameter["reference_frame"])
        # Planning frame is /base_footprint
        self.group.set_workspace(self.parameter["workspace"])
        self.group.set_goal_tolerance(self.parameter["tolerance"])
        rospy.logdebug("MoveitInterface.__init__(): Reference Frame is %s",
                       self.group.get_pose_reference_frame())
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=10)
        self.eef_link = self.parameter["eef_link"]
        self.touch_links = self.parameter["touch_links"]
        self.attached_equipment = None

    def _set_start_state(self):
        """
        Define the start state for planning - intended to be most current state of the Robot
        :return: msg
        :rtype: moveit_msgs.msg.RobotState
        """
        msg = moveit_msgs.msg.RobotState()
        js = JointState()
        js.header.frame_id = self.group.get_planning_frame()
        js.header.stamp = rospy.Time.now()
        js.name = self.robot.get_joint_names(self.group.get_name())[0:6]
        js.position = self.group.get_current_joint_values()
        msg.joint_state = js
        aco = self.scene.get_attached_objects()
        if len(aco) != 0:  # checks if dictionary is empty
            msg.attached_collision_objects = [fill_message(moveit_msgs.msg.AttachedCollisionObject(),
                                                                       self.scene.get_attached_objects())]
        rospy.logdebug("MoveitInterface._set_start_state(): Type(msg.attachecd_collision_object) is %s",
                       type(msg.attached_collision_objects))
        rospy.logdebug("MoveitInterface._set_start_state():msg.attachecd_collision_object \n %s",
                       msg.attached_collision_objects)
        return msg

    def move_to_target(self, target, info=""):
        """
        Move to designated target by using MoveIt interface
        :param target: either end_effector pose of joint values of the target pose
        :type target: list or Pose
        :param info: short information about the context of the given pose
        :type info: str
        :return: -
        :rtype: -
        """
        self.group.clear_pose_targets()
        start = self._set_start_state()
        rospy.logdebug("MoveitInterface.move_to_target(): Current Joint value %s",
                       ["%.2f" % v for v in np.rad2deg(self.group.get_current_joint_values())]
                       )
        self.group.set_start_state(start)
        try:
            if type(target) is list:
                target_dict = dict()
                joint_name = self.robot.get_joint_names(self.group.get_name())[0:6]
                for j in range(len(target)):
                    target_dict[joint_name[j]] = np.deg2rad(target[j])
                rospy.logdebug("MoveitInterface.move_to_target(): Planning Joint target %s", target_dict)
                self.group.set_joint_value_target(target_dict)
            elif target is Pose or PoseStamped:
                rospy.logdebug("MoveitInterface.move_to_target(): Planning Pose target %s", target)
                self.group.set_pose_target(target, end_effector_link=self.eef_link)
            else:
                rospy.logwarn("MoveitInterface.move_to_target(): Illegal target type: %s", type(target))
                return
        except moveit_commander.MoveItCommanderException as ex:
            rospy.logerr("MoveitInterface.move_to_target(): MoveItCommanderException during planning: %s " % ex.message)
            rospy.loginfo("MoveitInterface.move_to_target("+info+"): IST %s" %
                          ["%.2f" % v for v in self.group.get_current_joint_values()])
            rospy.loginfo("MoveitInterface.move_to_target("+info+"):SOLL %s" %
                          ["%.2f" % v for v in np.deg2rad(target)])
            rospy.loginfo("MoveitInterface.move_to_target("+info+"): SET %s" %
                          ["%.2f" % v for v in self.group.get_joint_value_target()])
            rospy.loginfo("MoveitInterface.move_to_target("+info+"):NAME %s" % target_dict.keys())
            rospy.loginfo("MoveitInterface.move_to_target("+info+"): constraints %s" %
                          self.group.get_known_constraints())

            sys.exit(-1)
        plan = None
        plan_valid = False
        while not plan_valid:
            rospy.loginfo("MoveitInterface.move_to_target(): Planning "+info+" to %s",
                          target)
            plan = self.group.plan()
            if len(plan.joint_trajectory.points) == 0:
                rospy.loginfo("MoveitInterface.move_to_target(): No valid plan found, trying again ...")
                rospy.logdebug("MoveitInterface.move_to_target(): Current Joint values[deg] %s",
                               ["%.2f" % v for v in np.rad2deg(self.group.get_current_joint_values())]
                               )
                continue
            # # Check Plan
            rospy.loginfo("MoveitInterface.move_to_target(): Please confirm the plan using the interactive marker on "
                          "topic: '/confirm_plan_marker/markers/update'")
            plan_valid = wait_for_confirmation(service_ns="~confirm_plan", timeout=60)
            # Check Plan - inline
            # plan_valid = continue_by_console("Is robot allowed to execute presented plan?")
        rospy.loginfo("MoveitInterface.move_to_target(): Executing ...")
        if not self.group.execute(plan):
            rospy.logdebug("MoveitInterface.move_to_target(): Execution returned False")
            # if continue_by_console("Try again?"):
            if True:
                self.move_to_target(target, info=info)
        return

    def add_equipment(self, eq, pose = None):
        """
        Add the given equipment to the planning scene
        :param eq: Equipment
        :type eq: Equipment
        :return: -
        :rtype: -
        """
        if len(self.scene.get_attached_objects([eq.name])) != 0:
            rospy.logwarn("MoveitInterface.add_equipment(): %s was allready present in the scene, removing it and add "
                          "it again" % eq.name)
            self.scene.remove_attached_object(link=self.eef_link, name=eq.name)
        if pose is None:
            self.scene.add_mesh(name=eq.name, pose=eq.robot_pick_pose, filename=eq.mesh_path,
                                size=(eq.scale.x, eq.scale.y, eq.scale.z))
        else:
            self.scene.add_mesh(name=eq.name, pose=pose, filename=eq.mesh_path,
                                size=(eq.scale.x, eq.scale.y, eq.scale.z))

    def attach_equipment(self, equipment):
        """
        Attach the given equipment to the end effector
        :param equipment: grasp equipment
        :type equipment: Equipment
        :return: -
        :rtype: -
        """
        # http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html
        # Attaching an object requires two operations:
        #   - Removing the original object from the environment
        #   - Attaching the object to the robot
        # self.scene.remove_world_object(equipment.name)
        self.scene.attach_mesh(link=self.eef_link, name=equipment.name, touch_links=self.touch_links)
        self.attached_equipment = equipment

    def detach_equipment(self):
        """
        Detach equipment from the robot
        :return: former attached equipment, pose unknown to this class, but should be known to outer scope
        :rtype: Equipment
        """
        # http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html
        # Detaching an object from the robot requires two operations
        #   - Detaching the object from the robot
        #   - Re-introducing the object into the environment
        if self.attached_equipment is None:
            rospy.logwarn("MoveitInterface.detach_equipment: self.attached_equipment is %s" % self.attached_equipment)
            return
        self.scene.remove_attached_object(self.eef_link, self.attached_equipment.name)
        retValue = self.attached_equipment
        self.attached_equipment = None
        # The station is still present in the scene, but now as an environmental object. Tho it has to be readded the
        # planning scene. In order to do so its pose needs to ne determined and passed to the interface by calling the
        # add_equipment() method of this class. For easier programming the detached equipment is returned.
        return retValue

    def clear_octomap_on_marker(self, equipment):
        """
        Clears the octomap on a desired pose with a marker to enable planning to this pose without collision -
        use careful
        :param equipment: marker on the desired pose
        :type equipment: marker.SSBMarker
        :return: -
        :rtype: -
        """
        rospy.loginfo("MoveitInterface.clear_octomap_on_marker(): Equipment %s", equipment)
        p = parse_to_os_path(equipment.getMeshResourcePath())
        rospy.loginfo("WlanSetTask._clear_octomap_on_marker(): MeshPath %s", p)
        ps = PoseStamped(header=equipment.header, pose=equipment.pose)
        self.scene.add_mesh(name="tmp_marker", pose=ps, filename=p, size=equipment.getMeshScale())
        # Octomap should be cleared of obstacles where the marker is added, now remove it to prevent collision
        self.scene.remove_world_object(name="tmp_marker")
        return


class EquipmentTask(GraspTask):
    """
    Handle equipment using the gripper unit of Julius
    """

    def __init__(self):
        """
        Default constructor, start ROS, hand_model and demo_monitoring
        """

        # Init Moveit
        self.moveit = MoveitInterface("~moveit")
        # Equipment Parameter
        self.lst_equipment = []
        for equip in rospy.get_param("~equipment"):
            eq = Equipment(equip)
            self.moveit.add_equipment(eq)
            self.lst_equipment.append(eq)
        self.selected_equipment = self.lst_equipment[0]

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

        self.tf_listener = tf.TransformListener()
        rospy.loginfo("EquipmentTask.__init__(): initialized")

    def select_equipment(self, name="Smart Sensor Box"):
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
        source_frame = query_pose.header.frame_id
        target_frame = self.moveit.group.get_pose_reference_frame()
        rospy.loginfo("EquipmentTask.generate_goal(): query_pose.header.frame_id: \n %s", source_frame)
        rospy.loginfo("EquipmentTask.generate_goal(): moveit.group.get_pose_reference_frame(): \n %s", target_frame)
        self.tf_listener.waitForTransform(target_frame, source_frame, query_pose.header.stamp, rospy.Duration(1))
        try:
            ps = PoseStamped()
            ps.header = query_pose.header
            ps.header.frame_id = target_frame
            ps.pose = self.tf_listener.transformPose(target_frame=target_frame, ps=query_pose)
        except Exception as ex:
            rospy.logwarn(ex.message)
            rospy.logwarn("EquipmentTask.generate_goal():can't transform pose to desired frame: %s", target_frame)
            return None
        return ps

    def perform(self):
        """
        Equipment Handle Task:
            1. Scan Environment
            2. Pick Up Equipment
            3. Update Planning Scene
            4. Query Set Pose
            5. Set Equipment
            (6. Return Equipment)
            7. Return to Init State
        :return: -
        :rtype: -
        """
        rospy.loginfo("EquipmentTask.perform(): Equipment handling started - Robot starting at HOME position")
        # 0. Open hand, in case smart equipment is still stuck
        self.hand_controller.openHand()
        rospy.sleep(1.0)
        # 1. Scan Environment
        rospy.loginfo("EquipmentTask.perform(): Closing hand  and scan the environment by given watch pose")
        self.hand_controller.closeHand()
        self.moveit.move_to_target(self.home_joint_values, info="HOME")
        self.moveit.move_to_target(self.watch_joint_values, info="Watch Pose")

        # 2. Pick Up Equipment
        # TODO: Select Equipment
        self.moveit.move_to_target(self.selected_equipment.pick_waypoints["pre"], info="PreGrasp")
        rospy.loginfo("EquipmentTask.perform(): Opening hand ...")
        self.hand_controller.openHand()
        self.moveit.move_to_target(self.selected_equipment.pick_waypoints["grasp"], info="Grasp")
        rospy.loginfo("EquipmentTask.perform(): Grasp equipment")
        # Grasp station
        self.hand_controller.closeHand()
        rospy.sleep(5.)

        # 3. Update Planning Scene - Attach collision object to end effector
        rospy.loginfo("EquipmentTask.perform(): Attach equipment to end effector")
        self.moveit.attach_equipment(self.selected_equipment)
        self.moveit.move_to_target(self.selected_equipment.pick_waypoints["post"], info="PostGrasp")

        # 4. Query Goal from User Interface
        eq_set_pose = Pose()
        int_marker = marker.SSBMarker(pose=eq_set_pose)
        query_pose_topic = int_marker.get_pose_topic()
        query_pose_subscriber = message_filters.Subscriber(query_pose_topic, PoseStamped)
        query_pose_cache = message_filters.Cache(query_pose_subscriber, 5)
        query_pose = None
        while query_pose is None:
            query_pose = query_pose_cache.getLast()
            rospy.logdebug("EquipmentTask.perform(): Set equipment to pose: \n %s", query_pose)
            rospy.sleep(0.5)
        # Set station
        rospy.loginfo("EquipmentTask.perform(): Set equipment ...")
        # Formulate Planning Problem
        self.moveit.clear_octomap_on_marker(int_marker)
        target_pose = self.generate_goal(query_pose)
        rospy.loginfo("WlanSetTask.perform(): Query new SSB Pose")
        while target_pose is None:
            now = rospy.Time.now()
            rospy.sleep(1.0)
            query_pose = query_pose_cache.getLast()
            if query_pose.header.time > now:
                target_pose = self.generate_goal(query_pose)
            else:
                rospy.logdebug("WlanSetTask.perform(): No new SSB Pose yet")

        # Plan Path using Moveit
        self.moveit.move_to_target(target_pose.pose, info="SSB_Pose")

        rospy.loginfo("WlanSetTask.perform(): Release station")
        self.hand_controller.openHand()
        rospy.sleep(5.)
        self._setup_move_group()
        # Remove collision object from end effector
        self.mvit_scene.remove_attached_object(link=self.eef_link, name=self.ssb_name)
        # self._hand_ssb_broadcaster.stop()
        # self.eef_link = rospy.get_param("~eef_link", "gripper_robotiq_palm_planning")
        # self.move_wait(self.waypoints["set_down"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")

        rospy.loginfo("WlanSetTask.perform(): Return to home pose")
        # Plan back to home station
        self.move_to_target(self.waypoints["home_pose"], info="HOME")

        self.mvit_scene.remove_world_object(name=self.ssb_name)

    def start(self):
        """
        Start the equipment handle task
        :return: -
        :rtype: -
        """
        rospy.loginfo("GraspTask.start():")
        self.perform()
        # super(EquipmentTask, self).run_as_process(self.perform)


if __name__ == '__main__':
    rospy.init_node("EquipmentTask", log_level=rospy.INFO)
    obj = EquipmentTask()
    obj.start()
    rospy.spin()
