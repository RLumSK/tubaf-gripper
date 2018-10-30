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
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from tbf_gripper_tools.Equipment import Equipment
from tubaf_tools import parse_to_os_path, fill_message
from tubaf_tools.confirm_service import wait_for_confirmation
import tbf_gripper_rviz.ssb_marker as marker


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
        self.scene.remove_world_object()  # Removes all objects since no name is given
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
        self.max_attempts = self.parameter["max_attempts"]
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
        :param target: either end_effector pose of joint values of the target pose (assumed to be in the reference frame
         of the MoveIt planning group)
        :type target: list or Pose
        :param info: short information about the context of the given pose
        :type info: str
        :return: success
        :rtype: bool
        """
        retValue = False
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
            rospy.loginfo("MoveitInterface.move_to_target(" + info + "): IST %s" %
                          ["%.2f" % v for v in self.group.get_current_joint_values()])
            rospy.loginfo("MoveitInterface.move_to_target(" + info + "):SOLL %s" %
                          ["%.2f" % v for v in np.deg2rad(target)])
            rospy.loginfo("MoveitInterface.move_to_target(" + info + "): SET %s" %
                          ["%.2f" % v for v in self.group.get_joint_value_target()])
            rospy.loginfo("MoveitInterface.move_to_target(" + info + "):NAME %s" % target_dict.keys())
            rospy.loginfo("MoveitInterface.move_to_target(" + info + "): constraints %s" %
                          self.group.get_known_constraints())

            sys.exit(-1)
        plan = None
        plan_valid = False
        attempts = 0
        while not plan_valid and attempts < self.max_attempts:
            rospy.loginfo("MoveitInterface.move_to_target(): Planning " + info + " to: \n%s",
                          target)
            plan = self.group.plan()
            attempts += 1
            if len(plan.joint_trajectory.points) == 0:
                rospy.loginfo("MoveitInterface.move_to_target(): No valid plan found, trying again (%d/%d) ..." %
                              (attempts, self.max_attempts))
                rospy.logdebug("MoveitInterface.move_to_target(): Current Joint values[deg] %s",
                               ["%.2f" % v for v in np.rad2deg(self.group.get_current_joint_values())]
                               )
                continue
            # # Check Plan
            rospy.loginfo("MoveitInterface.move_to_target(): Please confirm the plan using the interactive marker on "
                          "topic: '/confirm_plan_marker/markers/update'")
            plan_valid = wait_for_confirmation(service_ns="~confirm_plan", timeout=60)

        if attempts >= self.max_attempts:
            # We can't plan to the specified target
            return False
        rospy.loginfo("MoveitInterface.move_to_target(): Executing ...")
        executed = self.group.execute(plan)
        retValue = executed
        if not executed:
            rospy.logdebug("MoveitInterface.move_to_target(): Execution returned False")
            # if continue_by_console("Try again?"):
            if True:
                retValue = self.move_to_target(target, info=info)
        return retValue

    def add_equipment(self, eq, pose=None):
        """
        Add the given equipment to the planning scene
        :param pose: add equipment at specified pose
        :type pose: PoseStamped
        :param eq: Equipment
        :type eq: Equipment
        :return: -
        :rtype: -
        """
        if len(self.scene.get_attached_objects([eq.name])) != 0:
            rospy.logwarn("MoveitInterface.add_equipment(): %s was allready present in the scene, removing it and add "
                          "it again" % eq.name)
            self.scene.remove_attached_object(link=self.eef_link, name=eq.name)
            rospy.sleep(0.5)
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
        rospy.loginfo("AttachEquip: to: {} equip: {}".format(self.eef_link, equipment.name))
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
