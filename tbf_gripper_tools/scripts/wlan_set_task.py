#!/usr/bin/python
# Software License Agreement (MIT License)
#
# Copyright (c) 2015, TU Bergakademie Freiberg
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
# author: grehl

import signal
import sys
import os
import threading

import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import message_filters
import tf
import tf.transformations as tft
from camera_info_manager import URL_empty, URL_file, URL_invalid, URL_package, resolveURL, parseURL, getPackageFileName


import autonomy.Task
import tubaf_tools
import numpy as np

import tbf_gripper_rviz.ssb_marker as ssb_marker


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(s, f):
    print('You pressed Ctrl+C! signal %s, frame: %s' % (s, f))
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


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
        topic = rospy.get_param("~continue_topic", "/wlan_set_task/continue")
    subscriber = message_filters.Subscriber(topic, std_msgs.msg.Bool)
    cache = message_filters.Cache(subscriber, 5, allow_headerless=True)
    while cache.getLast() is None:
        rospy.sleep(2.0)
    return cache.getLast().data


def parse_to_os_path(p):
    """
    Parse the given ROS Path to its OS equivalent
    :param p: Path
    :type p: String
    :return: Path
    :rtype: String
    """
    resolved_url = resolveURL(p, "")
    url_type = parseURL(resolved_url)
    if url_type == URL_empty:
        return p
    rospy.loginfo('[parse_to_os_path()] writing calibration data to URL: ' + resolved_url)
    if url_type == URL_file:
        return resolved_url[7:]
    elif url_type == URL_package:
        filename = getPackageFileName(resolved_url)
        if filename == '':  # package not resolved
            rospy.logerr('[parse_to_os_path()] Package missing: ' +
                         resolved_url + ' (ignored)')
            # treat it like an empty URL
            return resolved_url
        else:
            return filename
    else:
        rospy.logerr("Invalid camera calibration URL: " + resolved_url)
        # treat it like an empty URL
        return resolved_url


class WlanSetTask(autonomy.Task.SetTask):
    """
    Class to get a set a wlan station
    """
    def __init__(self, station_nr=1):
        """
        default constructor of the set wlan task
        """
        rospy.loginfo("WlanSetTask.__init__(): Initializing Task...")
        autonomy.Task.SetTask.__init__(self)
        self.waypoints = rospy.get_param("~waypoints_station_"+str(station_nr))
        self.exec_thread = None
        self._hand_ssb_broadcaster = None

        rospy.loginfo("WlanSetTask.__init__(): Initializing MoveIt...")
        try:
            # Initialize MoveIt!
            # https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/scripts/move_group_python_interface_tutorial.py
            moveit_commander.roscpp_initialize(sys.argv)
            # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
            self.mvit_robot = moveit_commander.RobotCommander()
            rospy.logdebug("WlanSetTask.__init__(): Planning frame is %s", self.mvit_robot.get_planning_frame())
            # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding
            #  the robot.
            self.mvit_scene = moveit_commander.PlanningSceneInterface()
            # Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints. In this
            # case the group is the joints in the arm.  This interface can be used to plan and execute motions on the
            # arm.
            self.mvit_group = moveit_commander.MoveGroupCommander("UR5")
            self._setup_move_group()
            self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=10)
            self.eef_link = rospy.get_param("~eef_link", "gripper_robotiq_palm_planning")
            rospy.loginfo("WlanSetTask.__init__() Adding Collision Object...")
            #  Add collision object to planning scene
            # Smart Sensor Box (SSB)
            ssb_default_pose = rospy.get_param("~ssb_default_pose", geometry_msgs.msg.Pose())
            self.ssb_default_ps = geometry_msgs.msg.PoseStamped()

            pos = geometry_msgs.msg.Point(**ssb_default_pose['position'])
            ori = geometry_msgs.msg.Quaternion(**ssb_default_pose['orientation'])

            self.ssb_default_ps.pose = geometry_msgs.msg.Pose(pos, ori)
            self.ssb_default_ps.header.frame_id = "base_footprint"

            ssb_mesh_filename = rospy.get_param("~ssb_mesh_filename", os.path.join(
                rospkg.RosPack().get_path('tbf_gripper_tools'), 'resources', 'mesh', 'wlan_box.stl'))
            x_scale = rospy.get_param("~ssb_x_scale", 1.0)
            y_scale = rospy.get_param("~ssb_y_scale", 1.0)
            z_scale = rospy.get_param("~ssb_z_scale", 1.0)
            self.ssb_name = "Smart_Sensor_Box"
            if len(self.mvit_scene.get_attached_objects([self.ssb_name])) != 0:
                self.mvit_scene.remove_attached_object(link=self.eef_link, name=self.ssb_name)
            # rospy.sleep(2.0)
            self.mvit_scene.add_mesh(name=self.ssb_name, pose=self.ssb_default_ps, filename=ssb_mesh_filename,
                                     size=(x_scale, y_scale, z_scale))
            # Water Sample Station (WSS)
            wss_default_pose = rospy.get_param("~wss_default_pose", geometry_msgs.msg.Pose())
            self.wss_default_ps = geometry_msgs.msg.PoseStamped()

            pos = geometry_msgs.msg.Point(**wss_default_pose['position'])
            ori = geometry_msgs.msg.Quaternion(**wss_default_pose['orientation'])

            self.wss_default_ps.pose = geometry_msgs.msg.Pose(pos, ori)
            self.wss_default_ps.header.frame_id = "water_sample_station_mount_link"

            wss_mesh_filename = rospy.get_param("~wss_mesh_filename", os.path.join(
                rospkg.RosPack().get_path('tbf_gripper_tools'), 'resources', 'mesh', 'water_station_scaled.stl'))
            x_scale = rospy.get_param("~wss_x_scale", 1.0)
            y_scale = rospy.get_param("~wss_y_scale", 1.0)
            z_scale = rospy.get_param("~wss_z_scale", 1.0)
            self.wss_name = "Water Sample Station"
            if len(self.mvit_scene.get_attached_objects([self.wss_name])) != 0:
                self.mvit_scene.remove_attached_object(link=self.eef_link, name=self.wss_name)
            # rospy.sleep(2.0)
            self.mvit_scene.add_mesh(name=self.wss_name, pose=self.wss_default_ps, filename=wss_mesh_filename,
                                     size=(x_scale, y_scale, z_scale))

        except Exception as ex:
            rospy.logwarn("[WlanSetTask.__init__()]: MoveIt failed to initalize")
            rospy.logwarn("[WlanSetTask.__init__()]: %s", ex.message)
            sys.exit()

        prefix = "gripper_robotiq_"
        self.touch_links = [
            prefix + "palm",
            prefix + "palm_planning",
            prefix + "finger_2_link_0",
            prefix + "finger_2_link_1",
            prefix + "finger_2_link_2",
            prefix + "finger_2_link_3",
            prefix + "finger_1_link_0",
            prefix + "finger_1_link_1",
            prefix + "finger_1_link_2",
            prefix + "finger_1_link_3",
            prefix + "finger_middle_link_0",
            prefix + "finger_middle_link_1",
            prefix + "finger_middle_link_2",
            prefix + "finger_middle_link_3"
        ]

        self.tf_listener = tf.TransformListener()

        rospy.loginfo("WlanSetTask.__init__() Initialized Task")

    def _setup_move_group(self):
        """
        Set Parameters of the Move Group "UR5"
        :return: -
        :rtype: -
        """
        self.mvit_group.set_planner_id(rospy.get_param("~planner_id", "KPIECEkConfigDefault"))
        self.mvit_group.set_planning_time(rospy.get_param("~planner_time", 10.0))
        self.mvit_group.set_num_planning_attempts(rospy.get_param("~planner_attempts", 5))
        self.mvit_group.allow_replanning(rospy.get_param("~allow_replanning", True))
        # [minX, minY, minZ, maxX, maxY, maxZ]
        self.mvit_group.set_pose_reference_frame(reference_frame=rospy.get_param("~reference_frame",
                                                                                 "/gripper_ur5_base_link"))
        # Planning frame is /base_footprint
        self.mvit_group.set_workspace(rospy.get_param("~workspace", [-1.6, -1.2, 0.0, 0.25, 1.2, 1.7]))
        self.mvit_group.set_goal_tolerance(rospy.get_param("~tolerance", 0.01))
        rospy.logdebug("WlanSetTask._setup_move_group(): Reference Frame is %s",
                       self.mvit_group.get_pose_reference_frame())

    def _set_start_state(self):
        """
        Define the start state for planning - intended to be most current state of the Robot
        :return: msg
        :rtype: moveit_msgs.msg.RobotState
        """
        msg = moveit_msgs.msg.RobotState()
        js = sensor_msgs.msg.JointState()
        js.header.frame_id = self.mvit_group.get_planning_frame()
        js.header.stamp = rospy.Time.now()
        js.name = self.mvit_robot.get_joint_names(self.mvit_group.get_name())[0:6]
        js.position = self.mvit_group.get_current_joint_values()
        msg.joint_state = js
        aco = self.mvit_scene.get_attached_objects()
        if len(aco) != 0:  # checks if dictionary is empty
            msg.attached_collision_objects = [tubaf_tools.fill_message(moveit_msgs.msg.AttachedCollisionObject(),
                                                                       self.mvit_scene.get_attached_objects())]
        rospy.logdebug("WlanSetTask._set_start_state(): Type(msg.attachecd_collision_object) is %s",
                       type(msg.attached_collision_objects))
        rospy.logdebug("WlanSetTask._set_start_state():msg.attachecd_collision_object \n %s",
                       msg.attached_collision_objects)
        # rospy.logdebug("WlanSetTask._set_start_state(): Start state\n %s", msg)
        return msg

    def _add_ssb_to_tf(self, ssb_name):
        """
        Add the attached object to the tf for better planning
        :param ssb_name: name of the object
        :type ssb_name: str
        :return: object link
        :rtype: str
        """
        cur_ps = geometry_msgs.msg.PoseStamped()
        cur_ps.header = self.ssb_default_ps.header
        cur_ps.pose = self.ssb_default_ps.pose
        while not self.tf_listener.canTransform(target_frame=self.eef_link, source_frame=cur_ps.header.frame_id,
                                                time=cur_ps.header.stamp):
            rospy.sleep(0.1)
        # [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]
        (tran, rot) = self.tf_listener.lookupTransform(target_frame=self.eef_link, source_frame=cur_ps.header.frame_id,
                                                       time=cur_ps.header.stamp)
        new_eef_link = ssb_name+"_link"
        # TfStaticBroadcaster[source_frame, target_frame, translation, rotation, rate=10.0]
        self._hand_ssb_broadcaster = tubaf_tools.EasyTfStaticBroadcaster(self.eef_link, new_eef_link, tran, rot)
        self._hand_ssb_broadcaster.start()
        return new_eef_link

    def _clear_octomap_on_marker(self, marker):
        """
        Clears the octomap on a desired pose with a marker to enable planning to this pose without collision -
        use careful
        :param marker: SSB marker on the desired pose
        :type marker: ssb_marker.SSBMarker
        :return: -
        :rtype: -
        """
        rospy.loginfo("WlanSetTask._clear_octomap_on_marker(): Marker %s", marker)
        p = parse_to_os_path(marker.getMeshResourcePath())
        rospy.loginfo("WlanSetTask._clear_octomap_on_marker(): MeshPath %s", p)
        ps = geometry_msgs.msg.PoseStamped(header=marker.header, pose=marker.pose)
        self.mvit_scene.add_mesh(name="tmp_marker", pose=ps, filename=p,
                                 size=marker.getMeshScale())
        # Octomap should be cleared of obstacles where the marker is added, now remove it to prevent collision
        self.mvit_scene.remove_world_object(name="tmp_marker")
        return

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
        self.mvit_group.clear_pose_targets()
        start = self._set_start_state()
        rospy.logdebug("WlanSetTask.move_to_target(): Current Joint value %s",
                       ["%.2f" % v for v in np.rad2deg(self.mvit_group.get_current_joint_values())]
                       )
        self.mvit_group.set_start_state(start)
        if type(target) is list:
            target_dict = dict()
            joint_name = self.mvit_robot.get_joint_names(self.mvit_group.get_name())[0:6]
            for j in range(len(target)):
                target_dict[joint_name[j]] = np.deg2rad(target[j])
            rospy.logdebug("WlanSetTask.move_to_target(): Planning Joint target %s", target_dict)
            self.mvit_group.set_joint_value_target(target_dict)
        elif target is geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped:
            rospy.logdebug("WlanSetTask.move_to_target(): Planning Pose target %s", target)
            self.mvit_group.set_pose_target(target, end_effector_link=self.eef_link)
        else:
            rospy.logwarn("WlanSetTask.move_to_target(): Illegal target type: %s", type(target))
            return
        plan = None
        plan_valid = False
        while not plan_valid:
            rospy.loginfo("WlanSetTask.move_to_target(): Planning "+info+" to %s",
                          target)
            plan = self.mvit_group.plan()
            if len(plan.joint_trajectory.points) == 0:
                rospy.loginfo("WlanSetTask.move_to_target(): No valid plan found, trying again ...")
                rospy.logdebug("WlanSetTask.move_to_target(): Current Joint values[deg] %s",
                               ["%.2f" % v for v in np.rad2deg(self.mvit_group.get_current_joint_values())]
                               )
                continue
            # # Check Plan
            # plan_valid = continue_by_topic()
            # Check Plan - inline
            plan_valid = continue_by_console("Is robot allowed to execute presented plan?")
        rospy.loginfo("WlanSetTask.move_to_target(): Executing ...")
        if not self.mvit_group.execute(plan):
            rospy.logdebug("WlanSetTask.move_to_target(): Execution returned False")
            if continue_by_console("Try again?"):
                self.move_to_target(target, info=info)
        return

    def perform(self):
        """
        Hard coded task to pickup a wlan box from the robot, set it on the ground and the return to a home pose
        :return: -
        :rtype: -
        """
        rospy.loginfo("WlanSetTask.perform(): Move to Station on top of the Robot starting at HOME position")
        # Move to Station on top of the Robot starting at HOME position
        rospy.loginfo("WlanSetTask.perform(): Closing hand ...")
        self.hand_controller.closeHand()
        self.move_to_target(self.waypoints["home_pose"], info="HOME")
        self.move_to_target(self.waypoints["watch_pose"], info="Watch Pose")
        self.move_to_target(self.waypoints["pre_grasp"], info="PreGrasp")
        rospy.loginfo("WlanSetTask.perform(): Opening hand ...")
        self.hand_controller.openHand()
        self.move_to_target(self.waypoints["grasp"], info="Grasp")

        rospy.loginfo("WlanSetTask.perform(): Grasp station")
        # Grasp station
        self.hand_controller.closeHand()
        # rospy.sleep(5.)
        # Attach collision object to end effector
        rospy.loginfo("WlanSetTask.perform(): Attach station to end effector")
        self.mvit_scene.attach_mesh(link=self.eef_link, name=self.ssb_name, touch_links=self.touch_links)
        self.move_to_target(self.waypoints["post_grasp"], info="PostGrasp")

        # Set station
        rospy.loginfo("WlanSetTask.perform(): Set station")
        # Query Goal from User Interface
        ssb_set_pose = geometry_msgs.msg.Pose()
        int_marker = ssb_marker.SSBMarker(pose=ssb_set_pose)
        station_pose_topic = int_marker.get_pose_topic()
        station_pose_subscriber = message_filters.Subscriber(station_pose_topic, geometry_msgs.msg.PoseStamped)
        station_pose_cache = message_filters.Cache(station_pose_subscriber, 5)
        station_pose = None
        while station_pose is None:
            station_pose = station_pose_cache.getLast()
            rospy.logdebug("WlanSetTask.perform(): Set station to pose: \n %s", station_pose)
            rospy.sleep(0.5)

        # Formulate Planning Problem
        self._clear_octomap_on_marker(int_marker)
        target_pose = self.generate_goal(station_pose)
        rospy.loginfo("WlanSetTask.perform(): Query new SSB Pose")
        while target_pose is None:
            now = rospy.Time.now()
            rospy.sleep(1.0)
            station_pose = station_pose_cache.getLast()
            if station_pose.header.time > now:
                target_pose = self.generate_goal(station_pose)
            else:
                rospy.logdebug("WlanSetTask.perform(): No new SSB Pose yet")

        # Plan Path using Moveit
        self.move_to_target(target_pose, info="SSB_Pose")

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

        self.exec_thread = None

    def generate_goal(self, ps, eef_frame=None):
        """
        Given a desired pose of the SSB, generate a target frame for the eef_link of the current kinematic chain
        :param ps: desired pose of the SSB
        :type ps: geometry_msgs.msg.PoseStamped
        :param eef_frame: end-effector frame name
        :type eef_frame: basestring
        :return: relating pose for the end-effector
        :rtype: geometry_msgs.msg.PoseStamped
        """
        rospy.logdebug("[WlanSetTask.generate_goal()] started")
        if eef_frame is None:
            eef_frame = self.mvit_group.get_end_effector_link()
            rospy.logdebug("[WlanSetTask.generate_goal()] eef_frame is %s", eef_frame)

        # Extract current transformation between end_effector and grabbed object
        # 1. Get grabbed object
        lst_obj = self.mvit_scene.get_attached_objects()
        if lst_obj is None or len(lst_obj) == 0:
            rospy.logwarn("[WlanSetTask.generate_goal()] It is assumed, that there is an object attached to the "
                          "end-effector for which the planing goal should be generated. Aborting!")
            rospy.logdebug("%s", lst_obj)
            return None
        aco = lst_obj[self.ssb_name]  # Added collision object
        rospy.logdebug("[WlanSetTask.generate_goal()] type(aco): %s\n link_name: %s\n header: %s\n mesh_poses: %s" %
                       (type(aco), aco.link_name, aco.object.header, aco.object.mesh_poses))

        # 2. Extract transformation from object to end-effector ... aco.object.mesh_poses[0]
        aco_link = aco.link_name
        obj_to_eef_transform = aco.object.mesh_poses[0]
        # Apply transform to target pose, in order to set the relative pose for the end-effector instead of the attached
        # object
        rospy.loginfo("[WlanSetTask.generate_goal()] Source pose:\n %s\nTranslation: %s\nRotation: %s" %
                      (ps, obj_to_eef_transform.position, obj_to_eef_transform.orientation))
        goal_pose = ps
        # goal_pose.header.frame_id = eef_frame - frame_id keeps the same, since the pose is relative to /base_footprint
        goal_pose.pose.position.x += obj_to_eef_transform.position.x
        goal_pose.pose.position.y += obj_to_eef_transform.position.y
        goal_pose.pose.position.z += obj_to_eef_transform.position.z
        quat = tft.quaternion_multiply([goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                                        goal_pose.pose.orientation.z, goal_pose.pose.orientation.w],
                                       [obj_to_eef_transform.orientation.x, obj_to_eef_transform.orientation.y,
                                        obj_to_eef_transform.orientation.z, obj_to_eef_transform.orientation.w])
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        rospy.logdebug("[WlanSetTask.generate_goal()] New pose:\n %s", goal_pose)
        # while not continue_by_console("Keep going?"):
        #     rospy.sleep(1.0)
        return goal_pose

    def start(self):
        """
        Start the wlan set task
        :return: -
        :rtype: -
        """
        rospy.loginfo("WlanSetTask.start():")
        self.run_as_process(WlanSetTask.perform)


if __name__ == '__main__':
    rospy.init_node("WlanSetTask", log_level=rospy.INFO)
    obj = None
    for i in range(1, 4):
        if obj is None or obj.exec_thread is None:
            obj = WlanSetTask(i)
            rospy.loginfo("wlan_set_task.py: main(): Start station: %d", i)
            obj.start()
        while obj.exec_thread is not None:
            rospy.sleep(0.5)
    rospy.spin()
