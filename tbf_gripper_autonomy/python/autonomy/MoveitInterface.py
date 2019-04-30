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
from moveit_commander import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from tf import TransformListener
from tubaf_tools import parse_to_os_path
from tubaf_tools.confirm_service import wait_for_confirmation
import tbf_gripper_rviz.ssb_marker as marker
from tbf_gripper_tools.SmartEquipment import SmartEquipment

from pyassimp.errors import AssimpError


class MoveitInterface(object):
    """
    Interface to the move_group wrapper
    """

    def __init__(self, param_group_name="~moveit", tf_listener=None):
        """
        Default constructor - Parameters are given via yaml file loaded onto the parameter server
        """
        self.parameter = rospy.get_param(param_group_name, dict())
        if tf_listener is None:
            tf_listener = TransformListener(rospy.Duration.from_sec(15.0))
        self.tf_listener = tf_listener
        # Initialize MoveIt!
        # https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/scripts/move_group_python_interface_tutorial.py
        moveit_commander.roscpp_initialize(sys.argv)
        # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()
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
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=10)
        self.eef_link = self.parameter["eef_link"]
        self.touch_links = self.parameter["touch_links"]
        self.max_attempts = self.parameter["max_attempts"]
        self.scene.remove_attached_object(link=self.eef_link)  # Remove any equipped item on the end effector
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
        dict_collision_objects = self.scene.get_attached_objects()
        for name, aco in dict_collision_objects.iteritems():
            rospy.logdebug("MoveitInterface._set_start_state(): %s is attached", name)
            msg.attached_collision_objects.append(aco)
        # if len(msg.attached_collision_objects) != 0:
        #     rospy.logdebug("MoveitInterface._set_start_state(): dict_collision_objects \n %s",
        #                    dict_collision_objects)
        #     for item in msg.attached_collision_objects:
        #         rospy.logdebug("MoveitInterface._set_start_state(): @msg \n %s", item)
        return msg

    def move_to_target(self, target, info=""):
        """
        Move to designated target by using MoveIt interface
        :param target: either end_effector pose of joint values of the target pose (assumed to be in the reference frame
         of the MoveIt planning group)
        :type target: list or Pose or PoseStamped
        :param info: short information about the context of the given pose
        :type info: str
        :return: success
        :rtype: bool
        """
        retValue = False
        self.group.clear_pose_targets()
        start = self._set_start_state()
        # rospy.logdebug("MoveitInterface.move_to_target(): Current Joint value %s",
        #                ["%.2f" % v for v in np.rad2deg(self.group.get_current_joint_values())]
        #                )
        self.group.set_start_state(start)
        try:
            if type(target) is list:
                target_dict = dict()
                joint_name = self.robot.get_joint_names(self.group.get_name())[0:6]
                for j in range(len(target)):
                    target_dict[joint_name[j]] = np.deg2rad(target[j])
                rospy.logdebug("MoveitInterface.move_to_target(): Planning Joint target %s", target_dict)
                self.group.set_joint_value_target(target_dict)
            elif type(target) is Pose:
                rospy.logwarn("MoveitInterface.move_to_target(): Planning Pose target %s", target)
                self.group.set_pose_target(target, end_effector_link=self.eef_link)
            elif type(target) is PoseStamped:
                lst_joint_target = []
                rospy.loginfo("MoveitInterface.move_to_target():")
                rospy.logdebug("NAME: IST; SOLL = SET")
                for name, ist, soll in zip(self.group.get_active_joints(), self.group.get_current_joint_values(),
                                           self.get_ik(target)):
                    # We want to have an joint angle between -180° and 180°
                    # Therefore we first add 180°, transform it to[0, 360°] and then substract 180°
                    solu = np.rad2deg(((soll+np.pi) % 2*np.pi)-np.pi)
                    lst_joint_target.append(solu)
                    rospy.logdebug("%s: %4.2f; %4.2f = %4.2f" % (name, ist, soll, solu))
                self.move_to_target(target=lst_joint_target)
                #target_frame = self.group.get_pose_reference_frame()
                #self.get_ik(target)
                #rospy.logdebug("MoveitInterface.move_to_target(): Transforming PoseStamped:\n %s\n Target Frame is %s" %
                #               (target, target_frame))
                #self.tf_listener.waitForTransform(target_frame=target_frame,
                #                                  source_frame=target.header.frame_id,
                #                                  time=rospy.Time.now(),
                #                                  timeout=rospy.Duration(5.0))
                #ps = self.tf_listener.transformPose(target_frame=target_frame, ps=target)
                #self.move_to_target(ps.pose, info)
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
            rospy.logdebug("MoveitInterface.move_to_target(): Plan:\n%s", plan)
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

    def add_equipment(self, equipment, pose=None):
        """
        Add the given equipment to the planning scene
        :param pose: add equipment at specified pose
        :type pose: PoseStamped
        :param equipment: SmartEquipment
        :type equipment: SmartEquipment
        :return: -
        :rtype: -
        """
        known_objects = self.scene.get_known_object_names()
        # rospy.logdebug("MoveitInterface.add_equipment(): Previous known objects %s", known_objects)
        if equipment.name in known_objects:
            # rospy.logdebug("MoveitInterface.add_equipment(): Already known:  %s", equipment.name)
            if equipment.name in self.scene.get_attached_objects():
                rospy.loginfo("MoveitInterface.add_equipment(): Detaching %s", equipment.name)
                self.scene.remove_attached_object(link=self.eef_link, name=equipment.name)
            return
        if pose is None:
            pose = equipment.ps

        try:
            rospy.logdebug("MoveitInterface.add_equipment(): Adding %s to the scene", equipment.name)
            self.scene.add_mesh(name=equipment.name, pose=pose, filename=equipment.mesh_path, size=(1.0, 1.0, 1.0))
        except AssimpError as ex:
            rospy.logwarn("MoveitInterface.add_equipment(): Exception of type: %s says: %s" % (type(ex), ex.message))
            rospy.logwarn("MoveitInterface.add_equipment(): Can't add %s with mesh_url: %s" % (equipment.name, equipment.mesh_path))

    def remove_equipment(self, name):
        """
        Remove an object with the given name from the planning scene
        :param name: name of the object
        :type name: str
        :return: -
        :rtype: -
        """
        lst_names = self.scene.get_known_object_names()
        if name in lst_names:
            lst_attached_obj = self.scene.get_attached_objects()
            if name in lst_attached_obj:
                self.scene.remove_attached_object(self.eef_link, name)
            else:
                self.scene.remove_world_object(name)
        else:
            rospy.logwarn("MoveitInterface.remove_equipment(): %s is not present in the scene and hence can't be removed"
                          " known objects are: %s" % (name, lst_names))

    def attach_equipment(self, equipment):
        """
        Attach the given equipment to the end effector
        :param equipment: grasp equipment
        :type equipment: SmartEquipment
        :return: -
        :rtype: -
        """
        # http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html
        # Attaching an object requires two operations:
        #   - Removing the original object from the environment
        #   - Attaching the object to the robot
        rospy.loginfo("MoveitInterface.attach_equipment: {} equip: {}".format(self.eef_link, equipment.name))
        self.scene.remove_world_object(equipment.name)
        rospy.sleep(2.0)
        self.scene.attach_mesh(link=self.eef_link, name=equipment.name, filename=equipment.mesh_path,
                               pose=equipment.ps, touch_links=self.touch_links)
        self.attached_equipment = equipment

    def detach_equipment(self):
        """
        Detach equipment from the robot
        :return: former attached equipment, pose unknown to this class, but should be known to outer scope
        :rtype: SmartEquipment
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
        rospy.logdebug("MoveitInterface.clear_octomap_on_marker(): Equipment %s", equipment)
        p = parse_to_os_path(equipment.getMeshResourcePath())
        rospy.logdebug("MoveitInterface._clear_octomap_on_marker(): MeshPath %s", p)
        ps = PoseStamped(header=equipment.header, pose=equipment.pose)
        self.scene.add_mesh(name="tmp_marker", pose=ps, filename=p, size=equipment.getMeshScale())
        # Octomap should be cleared of obstacles where the marker is added, now remove it to prevent collision
        self.scene.remove_world_object(name="tmp_marker")
        return

    def get_object_pose(self, name):
        """
        Get current pose of the object
        :param name: name of the object
        :type name: str
        :return: pose of the object
        :rtype: PoseStamped
        """
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.group.get_pose_reference_frame()
        if name in self.scene.get_known_object_names():
            ps.pose = self.scene.get_object_poses(name)
        return ps

    def get_ik(self, ps, ik_link_name="gripper_robotiq_palm_planning"):
        """
        Compute an inverse kinematic using the service provided by MoveIt!
        :param ps: Desired pose of the end-effector
        :type ps: PoseStamped
        :param ik_link_name: name of the end effector link (used to interpret the given pose)
        :type ik_link_name: str
        :return: joint states computed by the IK if any
        :rtype: list
        """
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29#rospy_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.The_Code-1
        srv_name = 'compute_ik'
        rospy.wait_for_service(srv_name)
        response = GetPositionIKResponse()
        try:
            srv_call = rospy.ServiceProxy(srv_name, GetPositionIK)
            request = GetPositionIKRequest()
            request.ik_request.group_name = self.group.get_name()
            request.ik_request.robot_state = self.robot.get_current_state()  # type: RobotState
            request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = ik_link_name
            request.ik_request.pose_stamped = ps
            request.ik_request.timeout = rospy.Duration(10.0)
            request.ik_request.attempts = 50000
            response = srv_call(request)  # type: GetPositionIKResponse
        except rospy.ServiceException, e:
            rospy.logwarn("MoveitInterface.get_ik(): Service call failed: %s", e)
        if response.error_code.val != 1:
            rospy.logwarn("MoveitInterface.get_ik(): Failed with error code %s", response.error_code)
            return None
        # Filter joints
        ret_values = []
        active_joints = self.group.get_active_joints()
        for js_name, js_pos in zip(response.solution.joint_state.name, response.solution.joint_state.position):
            if js_name in active_joints:
                ret_values.append(js_pos)
        return ret_values


if __name__ == '__main__':
    rospy.init_node("MoveIt_Interface", log_level=rospy.INFO)
    # Init Moveit
    obj = MoveitInterface("~moveit")
    # Equipment Parameter
    for equip in rospy.get_param("~smart_equipment"):
        eq = SmartEquipment(equip)
        obj.add_equipment(eq)
