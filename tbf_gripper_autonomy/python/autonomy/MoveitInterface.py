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
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander import RobotState, RobotTrajectory
from moveit_msgs.msg import CollisionObject, Constraints, PlanningScene
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import numpy as np

from shape_msgs.msg import Mesh
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from tf import TransformListener
from tubaf_tools.confirm_service import wait_for_confirmation
from tbf_gripper_tools.SmartEquipment import SmartEquipment

import pyassimp
from pyassimp.errors import AssimpError

from evaluation.EvaluateEquipmentTask import EquipmentTask as Evaluation


def convert_angle(ist, sol, interval=360.0):
    """
    Given the current angle an desired one, compute the closest equivalent joint value
    :param ist: current joint value
    :type ist: float
    :param sol: desired joint value
    :type sol: float
    :param interval: full circle - 360 for degree and 2*pi for radian - default is 360.0
    :return: equivalent angle
    :rtype: float
    """
    diff = sol - ist
    rest = diff % interval
    sol1 = ist + rest
    sol2 = ist + rest - interval

    if abs(sol1 - ist) < abs(sol2 - ist) and -interval / 2.0 <= sol1 <= interval / 2.0:
        retval = sol1
    elif -interval / 2.0 <= sol2 <= interval / 2.0:
        retval = sol2
    else:
        retval = sol
    # rospy.logdebug("convert_angle(%g, %g, 360) -> %g oder %g -> %g" % (ist, sol, sol1, sol2, retval))
    return retval


# Ensuring Collision Updates Are Received
def wait_till_updated(pl_scene, obj_name, attached, known):
    """
    Wait until the planning scene was updated
    :param pl_scene: planning scene
    :param obj_name: name of the object to be updated
    :param attached: is the object now attached
    :param known: is the object now known
    :return: -
    """
    start = rospy.get_time()
    seconds = rospy.get_time()
    timeout = 1.0
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = pl_scene.get_attached_objects([obj_name])
        is_attached = len(attached_objects.keys()) > 0
        rospy.logdebug("[wait_till_updated()] Attached Objects: %s" % attached_objects.keys())

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = obj_name in pl_scene.get_known_object_names()
        rospy.logdebug("[wait_till_updated()] Known Objects: %s" % pl_scene.get_known_object_names())

        # Test if we are in the expected state
        if (attached == is_attached) and (known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.logdebug("[wait_till_updated()] Waiting for %s to be updated at planning scene ..." % obj_name)
        rospy.sleep(1.0)
        seconds = rospy.get_time()
    rospy.logwarn("[wait_till_updated(%s, attached=%s, known=%s)] Timeout reached" % (obj_name, attached, known))
    return False


class MoveitInterface(object):
    """
    Interface to the move_group wrapper
    """

    dct_moveit_error = {
        1: "Success",
        99999: "Failure",
        -1: "PLANNING_FAILED",
        -2: "INVALID_MOTION_PLAN",
        -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        -4: "CONTROL_FAILED",
        -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
        -6: "TIMED_OUT",
        -7: "PREEMPTED",
        -10: "START_STATE_IN_COLLISION",
        -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
        -12: "GOAL_IN_COLLISION",
        -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
        -14: "GOAL_CONSTRAINTS_VIOLATED",
        -15: "INVALID_GROUP_NAME",
        -16: "INVALID_GOAL_CONSTRAINTS",
        -17: "INVALID_ROBOT_STATE",
        -18: "INVALID_LINK_NAME",
        -19: "INVALID_OBJECT_NAME",
        -21: "FRAME_TRANSFORM_FAILURE",
        -22: "COLLISION_CHECKING_UNAVAILABLE",
        -23: "ROBOT_STATE_STALE",
        -24: "SENSOR_INFO_STALE",
        -31: "NO_IK_SOLUTION",
    }

    def __init__(self, param_group_name="~moveit", tf_listener=None, evaluation=False):
        """
        Default constructor - Parameters are given via yaml file loaded onto the parameter server
        """
        self.evaluation = evaluation  # type: Evaluation

        self.parameter = rospy.get_param(param_group_name, dict())
        if tf_listener is None:
            tf_listener = TransformListener(rospy.Duration.from_sec(15.0))
        self.tf_listener = tf_listener
        # Initialize MoveIt!
        # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#start-rviz-and-movegroup-node
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
        if len(self.parameter.keys()) > 0:
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
            self.lst_planner = self.parameter["planner"]
            self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=10)
            self.eef_link = self.parameter["eef_link"]
            self.touch_links = self.parameter["touch_links"]
            self.max_attempts = self.parameter["max_attempts"]
            self.ssb_scale = self.parameter["ssb_scale"]
            # https://groups.google.com/g/moveit-users/c/h75nDpwOKLk/m/1-IytpO_BQAJ?pli=1
            self.use_approximate_ik = self.parameter["use_approximate_ik"]
        else:
            self.group = moveit_commander.MoveGroupCommander("UR5")
            self.group.set_planner_id("KPIECEkConfigDefault")
            self.group.set_planning_time(1.0)
            self.group.set_num_planning_attempts(100)
            self.group.set_goal_tolerance(0.02)
            self.group.set_goal_position_tolerance(0.005)
            self.group.set_goal_orientation_tolerance(0.005)
            self.group.set_goal_joint_tolerance(0.01)
            self.group.allow_replanning(False)
            self.group.allow_looking(False)
            # [minX, minY, minZ, maxX, maxY, maxZ]
            self.group.set_pose_reference_frame(reference_frame="gripper_ur5_base_link")
            # Planning frame is /base_footprint
            self.group.set_workspace([-1.6, -1.2, 0.0, 0.25, 1.2, 1.7])
            self.group.set_goal_tolerance(0.005)
            self.lst_planner = ["RRTConnectkConfigDefault", "PRMstarkConfigDefault"]
            self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=10)
            self.eef_link = "gripper_robotiq_palm_planning"
            self.touch_links = []
            self.max_attempts = 5
            self.ssb_scale = 1.0
            self.use_approximate_ik = False
        self.scene.remove_attached_object(link=self.eef_link)  # Remove any equipped item on the end effector
        self.attached_equipment = None

        self.scene_diff_pub = rospy.Publisher("/planning_scene", data_class=PlanningScene, queue_size=1)

    def _remove_world_object(self, name=None):
        """
        Due to an missing frame_id in self.scene.remove_world_object(), we implement it ourself
        Remove an object from planning scene, or all if no name is provided
        """
        co = CollisionObject()
        co.header.frame_id = "base_footprint"
        co.operation = CollisionObject.REMOVE
        if name is not None:
            co.id = name
        self.scene._pub_co.publish(co)

    def _set_start_state(self):
        """
        Define the start state for planning - intended to be most current state of the Robot
        :return: msg
        :rtype: moveit_msgs.msg.RobotState
        """
        msg = moveit_msgs.msg.RobotState()  # type: RobotState
        # rospy.logdebug("MoveitInterface._set_start_state(): planning frame: %s ", self.group.get_planning_frame())
        js = JointState()
        js.header.frame_id = self.group.get_planning_frame()
        js.header.stamp = rospy.Time.now()
        js.name = self.robot.get_joint_names(self.group.get_name())[0:6]
        js.position = self.group.get_current_joint_values()
        msg.joint_state = js
        dict_collision_objects = self.scene.get_attached_objects()
        for name, aco in dict_collision_objects.items():  # iteritems
            rospy.logdebug("MoveitInterface._set_start_state(): %s is attached", name)
            msg.attached_collision_objects.append(aco)
        # if len(msg.attached_collision_objects) != 0:
        #     rospy.logdebug("MoveitInterface._set_start_state(): dict_collision_objects \n %s",
        #                    dict_collision_objects)
        #     for item in msg.attached_collision_objects:
        #         rospy.logdebug("MoveitInterface._set_start_state(): @msg \n %s", item)
        # rospy.logdebug("MoveitInterface._set_start_state(): msg %s ", msg)
        return msg

    def wait_till_updated(self, obj_name, attached, known):
        """
        Wait until the planning scene was updated
        :param obj_name: name of the object to be updated
        :param attached: is the object now attached
        :param known: is the object now known
        :return: -
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 5.0
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([obj_name])
            is_attached = len(attached_objects.keys()) > 0
            rospy.logdebug("[wait_till_updated()] Attached Objects: %s" % attached_objects.keys())

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = obj_name in self.scene.get_known_object_names()
            rospy.logdebug("[wait_till_updated()] Known Objects: %s" % self.scene.get_known_object_names())

            # Test if we are in the expected state
            if (attached == is_attached) and (known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.logdebug("[wait_till_updated()] Waiting for %s to be updated at planning scene ..." % obj_name)
            rospy.sleep(1.0)
            seconds = rospy.get_time()
        rospy.logwarn("[wait_till_updated(%s, attached=%s, known=%s)] Timeout reached" % (obj_name, attached, known))
        return False

    def plan(self, target, info="", blind=False, constraints=None):
        """
        Plan a trajectory towards the target using MoveIt
        :param blind: [DANGEROUS] if true, plan is executed without confirmation
        :param target: either end_effector pose of joint values (in deg) of the target pose (assumed to be in the
         reference frame of the MoveIt planning group)
        :type target: list or Pose or PoseStamped
        :param info: short information about the context of the given pose
        :type info: str
        :param constraints: constraints for the planning interface
        :type constraints: Constraints
        :return: plan
        :rtype: RobotTrajectory
        """
        self.group.clear_pose_targets()
        start = self._set_start_state()
        current_values = self.group.get_current_joint_values()  # type: list
        rospy.logdebug("MoveitInterface.plan(): Current Joint value %s",
                       ["%.2f" % v for v in np.rad2deg(current_values)]
                       )
        self.group.set_start_state(start)
        target_dict = dict()
        set_values = [0, 0, 0, 0, 0, 0]
        try:
            if type(target) is list:
                joint_name = self.robot.get_joint_names(self.group.get_name())[0:6]
                for j, ist in zip(range(len(target)), current_values):
                    target_dict[joint_name[j]] = np.deg2rad(convert_angle(np.rad2deg(ist), target[j]))
                    set_values[j] = target_dict[joint_name[j]]
                    rospy.logdebug("MoveitInterface.plan(): %s \t%4.2f (%4.2f) ->\t%4.2f" % (joint_name[j], target[j],
                                                                                             np.rad2deg(ist),
                                                                                             np.rad2deg(
                                                                                                 target_dict[
                                                                                                     joint_name[j]])))
                    self.group.set_joint_value_target(joint_name[j], target_dict[joint_name[j]])
            elif type(target) is Pose:
                rospy.logwarn("MoveitInterface.plan(): Planning Pose target %s", target)
                self.group.set_joint_value_target(target, self.eef_link, self.use_approximate_ik)
            elif type(target) is PoseStamped:
                # rospy.logdebug("MoveitInterface.plan(): Transforming PoseStamped target %s", target)
                lst_joint_target = []
                active_joints = self.group.get_active_joints()
                target_values = self.get_ik(target)  # type: list
                if target_values is None:
                    rospy.loginfo("MoveitInterface.plan(): IK Service call failed - give it a try using the "
                                  "pose interface")
                    self.group.set_joint_value_target(target, self.eef_link, self.use_approximate_ik)
                else:
                    for name, ist, soll in zip(active_joints, current_values, target_values):
                        solu = convert_angle(np.rad2deg(ist), np.rad2deg(soll))
                        lst_joint_target.append(solu)
                    # rospy.logdebug("MoveitInterface.plan(): move_to_target %s", lst_joint_target)
                    return self.plan(target=lst_joint_target, info=info, blind=blind)
            else:
                rospy.logwarn("MoveitInterface.plan(): Illegal target type: %s", type(target))
                return
        except moveit_commander.MoveItCommanderException as ex:
            rospy.logerr("MoveitInterface.plan(): MoveItCommanderException during planning: %s " % ex.message)
            current_joints = self.group.get_current_joint_values()
            target_values = self.group.get_joint_value_target()
            target_values.reverse()
            rospy.logerr("MoveitInterface.plan(" + info + "): current_joints %s" % current_joints)
            rospy.logerr("MoveitInterface.plan(" + info + "): set_values %s" % set_values)
            rospy.logerr("MoveitInterface.plan(" + info + "): target_values %s" % target_values)
            rospy.logerr("MoveitInterface.plan(" + info + "): constraints %s" % self.group.get_known_constraints())
            rospy.logerr("MoveitInterface.plan(" + info + "): target type: %s" % type(target))
            return False

        if constraints is not None:
            rospy.loginfo("MoveitInterface.plan(): Setting %s constraints" % constraints.name)
            self.group.set_path_constraints(constraints)

        plan = None
        plan_valid = False
        attempts = 1
        while not plan_valid and attempts <= self.max_attempts:
            rospy.loginfo("MoveitInterface.plan(): Planning %s to: \n%s\tPlanning time: %s" %
                          (info, target, self.group.get_planning_time()))
            # HERE WE PLAN #
            plan = self.group.plan()

            self.group.set_planning_time(self.parameter["planner_time"] * attempts ** 2)
            self.group.set_num_planning_attempts(self.parameter["planner_attempts"] * attempts ** 2)
            attempts += 1
            # rospy.logdebug("MoveitInterface.move_to_target(): Plan:\n%s", plan)
            if len(plan.joint_trajectory.points) == 0:
                rospy.loginfo("MoveitInterface.plan(): No valid plan found, trying again (%d/%d) ..." %
                              (attempts, self.max_attempts + 1))
                rospy.logdebug("MoveitInterface.plan(): Current Joint values[deg] %s",
                               ["%.2f" % v for v in np.rad2deg(self.group.get_current_joint_values())]
                               )
                continue
            # # Check Plan
            if blind:
                rospy.loginfo("MoveitInterface.plan(" + info + "): Not waiting for confirmation")
                break
            rospy.loginfo("MoveitInterface.plan(): Please confirm the plan using the interactive marker on "
                          "topic: '/confirm_plan_marker/markers/update'")
            if self.evaluation:
                self.evaluation.pause()
            plan_valid = wait_for_confirmation(service_ns="~confirm_plan", timeout=60)
            if self.evaluation:
                self.evaluation.resume()

        self.group.set_planning_time(self.parameter["planner_time"])
        self.group.set_num_planning_attempts(self.parameter["planner_attempts"])
        if attempts >= self.max_attempts:
            # We can't plan to the specified target
            return False
        if self.evaluation and plan_valid:
            try:
                self.evaluation.add_moveit_plan_information(info, plan, self.group.get_planning_time(),
                                                            attempts, self.evaluation.calc_time(now=rospy.Time.now()))
            except KeyError as ke:
                rospy.logerr("[MoveitInterface.plan()] Evaluation - KeyError: %s" % ke)
        return plan

    def execute(self, plan):
        """
        Move to designated target by using MoveIt interface
        :param plan: plan defiened by MoveitInterface.plan()
        :type plan: RobotTrajectory
        :return: success
        :rtype: bool
        """
        if plan is None:
            rospy.logwarn("MoveitInterface.execute(): Plan is None")
            return False
        elif plan is False:
            return False
        rospy.loginfo("MoveitInterface.execute(): Executing ...")
        return self.group.execute(plan)

    def move_to_set(self, target, info, endless=True, constraints=None):
        """
        Same as move_to_target() but while ignoring the planing scene
        :param endless: loop the computation of a plan/trajectory
        :type endless: bool
        :type target: list or Pose or PoseStamped
        :param info: short information about the context of the given pose
        :type info: str
        :param constraints: List of contraints for each joint configuration in the path
        :type constraints: Constraints
        :return: success
        :rtype: bool
        """
        from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, GetPlanningSceneResponse
        from moveit_msgs.msg import PlanningSceneComponents
        from std_srvs.srv import Empty

        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        response = get_planning_scene(
            components=PlanningSceneComponents(
                components=PlanningSceneComponents.OCTOMAP))  # type: GetPlanningSceneResponse
        current_octomap = response.scene  # type:PlanningScene
        current_octomap.is_diff = True  # keeps robot_state, since we will only add the old octomap
        rospy.logdebug("[MoveitInterface.move_to_set()] current_octomap id: %s \t resolution: %s" % (
            current_octomap.world.octomap.octomap.id, current_octomap.world.octomap.octomap.resolution))
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()
        self.move_to_target(target, info, endless, constraints=constraints)
        success = apply_planning_scene(current_octomap)
        rospy.logdebug("[MoveitInterface.move_to_set()] apply_planning_scene successful? %s" % success)
        return success

    def move_to_target(self, target, info, endless=True, blind=False, constraints=None):
        """
        Plan and execute
        :param endless: loop the computation of a plan/trajectory
        :param blind: [DANGEROUS] if true, plan is executed without confirmation
        :param target: either end_effector pose of joint values (in deg) of the target pose (assumed to be in the
        reference frame of the MoveIt planning group)
        :type target: list or Pose or PoseStamped
        :param info: short information about the context of the given pose
        :type info: str
        :param constraints: List of contraints for each joint configuration in the path
        :type constraints: Constraints
        :return: success
        :rtype: bool
        """
        plan = False
        success = False
        iplanner = 0
        try:
            while not plan:
                plan = self.plan(target, info, blind=blind, constraints=constraints)
                if plan:
                    if self.evaluation:
                        try:
                            if info in self.evaluation.dct_planner:
                                self.evaluation.dct_planner.append(self.lst_planner[iplanner])
                            else:
                                self.evaluation.dct_planner = [self.lst_planner[iplanner]]
                        except Exception as ex:
                            rospy.logerr("[MoveitInterface.move_to_target] Exception on evaluation %s" % ex.message)
                    success = self.execute(plan)
                    rospy.loginfo("[MoveitInterface.move_to_target] Finished %s motion" % info)
                if not endless or success:
                    return success
                if not success and endless:
                    new_planner = self.lst_planner[iplanner]
                    rospy.loginfo("[MoveitInterface.move_to_target] Changing planner to Nr. %s %s" % (iplanner,
                                                                                                      new_planner))
                    iplanner += 1
                    if iplanner >= len(self.lst_planner):
                        iplanner = 0
                    self.group.set_planner_id(new_planner)

                rospy.loginfo("[MoveitInterface.move_to_target] Trying again ...")
                plan = success
        except Exception as ex:
            rospy.logerr("[MoveitInterface.move_to_target] Exception %s", ex)
            raise ex
        finally:
            self.group.set_planner_id(self.parameter["planner_id"])
        return False

    def add_mesh_to_scene(self, name, pose, filename, size):
        """
        Instead of PlaningSceneInterface.add_mesh
        :param name:
        :param pose:
        :param filename:
        :param size:
        :return:
        """
        ### From Moveit_commander.planing_scene_interface.py PlanningSceneInterface.__make_mesh()
        scale = size
        co = CollisionObject()
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise Exception("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise Exception("There are no faces in the mesh")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header.frame_id = pose.header.frame_id

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        from shape_msgs.msg import MeshTriangle
        if hasattr(first_face, '__len__'):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, 'indices'):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise Exception("Unable to build triangles from mesh due to mesh object structure")
        from geometry_msgs.msg import Point
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        pyassimp.release(scene)

        msg = PlanningScene()
        msg.world.collision_objects.append(co)
        msg.is_diff = True
        self.scene_diff_pub.publish(msg)

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
        ao_keys = self.scene.get_attached_objects().keys()
        ko_names = self.scene.get_known_object_names()
        rospy.logdebug("MoveitInterface.add_equipment(): Attached objects %s", ao_keys)
        if equipment.name in ao_keys:
            rospy.loginfo("MoveitInterface.add_equipment(): Detaching %s", equipment.name)
            while not self.wait_till_updated(equipment.name, attached=False, known=True):
                self.scene.remove_attached_object(link=self.eef_link, name=equipment.name)
            while not self.wait_till_updated(equipment.name, attached=False, known=False):
                self.scene.remove_world_object(name=equipment.name)

        rospy.logdebug("MoveitInterface.add_equipment(): Known objects %s", ko_names)
        # if equipment.name in ko_names:
        #     rospy.logdebug("MoveitInterface.add_equipment(): Already known:  %s", equipment.name)
        #     return

        if pose is None:
            pose = equipment.ps
        try:
            rospy.logdebug("MoveitInterface.add_equipment(): Adding %s to the scene", equipment.name)
            scale = self.ssb_scale
            while not wait_till_updated(self.scene, equipment.name, attached=False, known=True):
                self.add_mesh_to_scene(name=equipment.name, pose=pose, filename=equipment.mesh_path,
                                    size=(scale, scale, scale))
                # self.scene.add_mesh(name=equipment.name, pose=pose, filename=equipment.mesh_path,
                #                     size=(scale, scale, scale))
        except AssimpError as ex:
            rospy.logwarn("MoveitInterface.add_equipment(): Exception of type: %s says: %s" % (type(ex), ex.message))
            rospy.logwarn("MoveitInterface.add_equipment(): Can't add %s with mesh_url: %s" % (
                equipment.name, equipment.mesh_path))
        except Exception as e:
            rospy.logwarn("MoveitInterface.add_equipment(): Exception of type: %s says: %s" % (type(e), e.message))

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
                wait_till_updated(self.scene, name, False, True)
            self._remove_world_object(name)
            wait_till_updated(self.scene, name, False, False)
        else:
            rospy.logwarn(
                "MoveitInterface.remove_equipment(): %s is not present in the scene and hence can't be removed"
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
        # self.scene.remove_world_object(equipment.name) is
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.header.frame_id = equipment.ps.header.frame_id
        co.id = equipment.name
        self.scene._pub_co.publish(co)
        wait_till_updated(self.scene, equipment.name, False, False)
        self.scene.attach_mesh(link=self.eef_link, name=equipment.name, filename=equipment.mesh_path,
                               pose=equipment.ps, touch_links=self.touch_links)
        wait_till_updated(self.scene, equipment.name, True, True)
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
        dct_obj = self.scene.get_attached_objects([self.attached_equipment.name])
        attached_obj = dct_obj[self.attached_equipment.name]
        obj_pose = PoseStamped()
        obj_pose.header = attached_obj.object.header
        obj_pose.pose = attached_obj.object.mesh_poses[0]

        self.scene.remove_attached_object(attached_obj.link_name, attached_obj.object.id)
        released_equipment = self.attached_equipment
        released_equipment.ps = obj_pose
        self.attached_equipment = None
        # wait_till_updated(self.scene, attached_obj.object.id, attached=False, known=False)
        rospy.sleep(0.5)

        # The station is still present in the scene, but now as an environmental object. Tho it has to be re-added the
        # planning scene. In order to do so its pose needs to be determined and passed to the interface by calling the
        # add_equipment() method of this class.
        world_frame = rospy.get_param("~wolrd_frame", default="base_footprint")
        self.tf_listener.waitForTransform(world_frame, released_equipment.ps.header.frame_id,
                                          released_equipment.ps.header.stamp, rospy.Duration(10))
        released_equipment.ps = self.tf_listener.transformPose(world_frame, released_equipment.ps)
        self.add_equipment(released_equipment)
        return released_equipment

    def clear_octomap_on_mesh(self, ps, mesh):
        """
        Clears the octomap on a desired pose with a marker to enable planning to this pose without collision -
        use careful
        :param ps: pose of the mesh
        :type ps: PoseStamped
        :param mesh: path to mesh
        :type mesh: str
        :return: -
        :rtype: -
        """
        # Octomap should be cleared of obstacles where the marker is added, now remove it to prevent collision
        # Due to an missing frame_id in self.scene.remove_world_object(), we implement it ourself
        orginal_ps = copy.deepcopy(ps)
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.header.frame_id = ps.header.frame_id
        co.id = "tmp_scaled"
        scaling = 1.5
        sleep = 2.0
        # ps.pose.position.z = ps.pose.position.z + 0.1 * (1 - scaling)  # move slightly in the ground
        self.scene.add_mesh(name=co.id, pose=ps, filename=mesh, size=(scaling, scaling, scaling))
        rospy.sleep(sleep)
        self.scene._pub_co.publish(co)
        rospy.sleep(sleep)
        self.scene.add_mesh(name="tmp_orginal", pose=orginal_ps, filename=mesh, size=(1.0, 1.0, 1.0))
        rospy.sleep(sleep)
        co.id = "tmp_orginal"
        self.scene._pub_co.publish(co)
        rospy.sleep(sleep)
        return

    def clear_octomap_via_box_marker(self, ps=None):
        """
        Clear the octomap above the bos station using a marker
        :param ps: optional pose where the box should clear
        :type ps: PoseStamped
        :return: None
        """
        rospy.logdebug("MoveitInterface.clear_octomap_via_box_marker():")
        # size = (0.4, 0.6, 0.4)
        # size = [0.6, 0.4, 0.5]
        size = [0.75, 0.5, 0.5]
        if ps is None:
            ps = PoseStamped()
            ps.header.frame_id = "controlbox_structure_top_front_link"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = 0  # was: size[0]/2.0
            ps.pose.position.y = size[1] / 2.0
            ps.pose.position.z = size[2] / 2.0
            # If we exited the while loop without returning then we timed out
            return False
        # Adding Objects to the Planning Scene
        # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#start-rviz-and-movegroup-node
        name = "rubber"
        self.scene.add_box(name, ps, size)
        wait_till_updated(self.scene, name, attached=False, known=True)
        rospy.loginfo("[MoveitInterface.clear_octomap_via_box_marker()] Added")
        self.scene.attach_box(ps.header.frame_id, name)
        wait_till_updated(self.scene, name, attached=True, known=False)
        rospy.loginfo("[MoveitInterface.clear_octomap_via_box_marker()] Attached")

        rospy.sleep(1.0)

        self.scene.remove_attached_object(ps.header.frame_id, name)
        wait_till_updated(self.scene, name, attached=False, known=True)
        rospy.loginfo("[MoveitInterface.clear_octomap_via_box_marker()] Detached")

        # Note: The object must be detached before we can remove it from the world
        self.scene.remove_world_object(name)
        wait_till_updated(self.scene, name, attached=False, known=False)
        rospy.loginfo("[MoveitInterface.clear_octomap_via_box_marker()] Removed")

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

    def _filter_joint_states(self, lst_names, lst_positions):
        """
        Filter the active joint states from a list of names and positions given
        :param lst_names: list of joint names
        :type lst_names: list
        :param lst_positions: list of joint values (angles in radian)
        :type lst_positions: list
        :return: list with values of the active joints
        :rtype: list
        """
        # Filter joints
        ret_values = []
        active_joints = self.group.get_active_joints()
        for js_name, js_pos in zip(lst_names, lst_positions):
            if js_name in active_joints:
                ret_values.append(js_pos)
        return ret_values

    def get_ik(self, ps, ik_link_name="gripper_robotiq_palm_planning"):
        """
        Compute an inverse kinematic using the service provided by MoveIt!
        :param ps: Desired pose of the end-effector
        :type ps: PoseStamped
        :param ik_link_name: name of the end effector link (used to interpret the given pose)
        :type ik_link_name: str
        :return: joint states computed by the IK if any
        :rtype: list or None
        """
        # http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29#rospy_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.The_Code-1
        srv_name = 'compute_ik'
        rospy.wait_for_service(srv_name)
        response = GetPositionIKResponse()
        request = GetPositionIKRequest()  # type: GetPositionIKRequest
        for attempt in range(self.max_attempts):
            try:
                srv_call = rospy.ServiceProxy(srv_name, GetPositionIK)
                request.ik_request.group_name = self.group.get_name()
                request.ik_request.robot_state = self.robot.get_current_state()
                request.ik_request.avoid_collisions = True
                request.ik_request.ik_link_name = ik_link_name
                request.ik_request.pose_stamped = ps
                request.ik_request.timeout = rospy.Duration(attempt * attempt)
                request.ik_request.attempts = 0  # each attempt get the timeout, so total time = timeout * attempts
                response = srv_call(request)  # type: GetPositionIKResponse
            except rospy.ServiceException as e:
                rospy.logwarn("MoveitInterface.get_ik(): Service call failed: %s", e)
            if response.error_code.val != 1:
                rospy.logwarn("MoveitInterface.get_ik(): Failed with error: %s (%s/%s)" % (
                              MoveitInterface.dct_moveit_error[response.error_code.val], attempt, self.max_attempts))
                rospy.logdebug("MoveitInterface.get_ik(): Target Pose was:\n%s", request.ik_request.pose_stamped)
                rospy.logdebug("MoveitInterface.get_ik(): Joint States were:%s", np.rad2deg(self._filter_joint_states(
                    request.ik_request.robot_state.joint_state.name,
                    request.ik_request.robot_state.joint_state.position)))
                rospy.logdebug("MoveitInterface.get_ik(): Response was:\n%s", response)
            else:
                break
        if response.error_code.val != 1:
            return None
        return self._filter_joint_states(response.solution.joint_state.name, response.solution.joint_state.position)


if __name__ == '__main__':
    rospy.init_node("MoveIt_Interface", log_level=rospy.INFO)
    # Init Moveit
    obj = MoveitInterface("~moveit")
    # Equipment Parameter
    for equip in rospy.get_param("~smart_equipment"):
        eq = SmartEquipment(equip)
        obj.add_equipment(eq)
