#!/usr/bin/python
# Software License Agreement (BSD License)
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

import rospy, rospkg
import moveit_commander, moveit_msgs.msg, geometry_msgs.msg, std_msgs.msg, sensor_msgs.msg
import message_filters

import autonomy.Task
import tubaf_tools
import numpy as np


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


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
        # autonomy.Task.SetTask.__init__(self)
        self.waypoints = rospy.get_param("~waypoints_station_"+str(station_nr))
        self.exec_thread = None

        rospy.loginfo("WlanSetTask.__init__(): Initializing MoveIt...")
        try:
            # Initialize MoveIt!
            # see: https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/scripts/move_group_python_interface_tutorial.py
            moveit_commander.roscpp_initialize(sys.argv)
            ## Instantiate a RobotCommander object.  This object is an interface to
            ## the robot as a whole.
            self.mvit_robot = moveit_commander.RobotCommander()
            rospy.logdebug("WlanSetTask.__init__(): Planning frame is %s", self.mvit_robot.get_planning_frame())
            ## Instantiate a PlanningSceneInterface object.  This object is an interface
            ## to the world surrounding the robot.
            self.mvit_scene = moveit_commander.PlanningSceneInterface()
            ## Instantiate a MoveGroupCommander object.  This object is an interface
            ## to one group of joints.  In this case the group is the joints in the
            ## arm.  This interface can be used to plan and execute motions on the arm.
            self.mvit_group = moveit_commander.MoveGroupCommander("UR5")
            self._setup_move_group()
            self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=10)
            # rospy.sleep(10)

            rospy.loginfo("WlanSetTask.__init__() Adding Collision Object...")
            #  Add collision object to planning scene
            self.eef_link = rospy.get_param("~eef_link", "gripper_robotiq_palm_planning")
            ssb_default_pose = rospy.get_param("~ssb_default_pose", geometry_msgs.msg.Pose())
            ssb_default_ps = geometry_msgs.msg.PoseStamped()

            pos = geometry_msgs.msg.Point(**ssb_default_pose['position'])
            ori = geometry_msgs.msg.Quaternion(**ssb_default_pose['orientation'])

            ssb_default_ps.pose = geometry_msgs.msg.Pose(pos,ori)
            ssb_default_ps.header.frame_id = "base_footprint"

            ssb_mesh_filename = rospy.get_param("~ssb_mesh_filename", os.path.join(
                rospkg.RosPack().get_path('tbf_gripper_tools'), 'resources', 'mesh', 'wlan_box.stl'))
            x_scale = rospy.get_param("~ssb_x_scale", 1.0)
            y_scale = rospy.get_param("~ssb_y_scale", 1.0)
            z_scale = rospy.get_param("~ssb_z_scale", 1.0)
            self.ssb_name = "Smart_Sensor_Box"
            self.mvit_scene.remove_attached_object(link=self.eef_link, name=self.ssb_name)
            self.mvit_scene.add_mesh(name=self.ssb_name, pose=ssb_default_ps, filename=ssb_mesh_filename,
                                     size=(x_scale, y_scale, z_scale))
        except Exception as ex:
            rospy.logwarn("[WlanSetTask.__init__()]: MoveIt failed to initalize")
            rospy.logwarn("[WlanSetTask.__init__()]: %s", ex.message)
            sys.exit()

        ui_pose_topic = rospy.get_param("~ui_pose_topic", "/wlan_set_task/ui_pose")
        permission_topic = rospy.get_param("~permission_topic", "/wlan_set_task/permission")
        self.ui_pose_subscriber = message_filters.Subscriber(ui_pose_topic, geometry_msgs.msg.PoseStamped)
        self.ui_pose_cache = message_filters.Cache(self.ui_pose_subscriber, 5)
        self.ui_check_subscriber = message_filters.Subscriber(permission_topic, std_msgs.msg.Bool)
        self.ui_check_cache = message_filters.Cache(self.ui_check_subscriber, 5, allow_headerless=True)

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
        self.mvit_group.set_workspace(rospy.get_param("~workspace", [-1.6, -1.2, 0.0, 0.25, 1.2, 1.2]))
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

    def move_wait(self, target, info=""):
        """
        Move to designated target by using MoveIt interface
        :param target: either end_effector pose of joint values of the target pose
        :type target: list or Pose
        :param info: short information about the context of the given pose
        :type info: String
        :return: -
        :rtype: .
        """
        self.mvit_group.clear_pose_targets()
        start = self._set_start_state()
        rospy.logdebug("WlanSetTask.move_wait(): Current Joint value %s", self.mvit_group.get_current_joint_values())
        rospy.logdebug("WlanSetTask.move_wait(): type(target) %s", type(target))
        self.mvit_group.set_start_state(start)
        if type(target) is list:
            target_dict = dict()
            joint_name = self.mvit_robot.get_joint_names(self.mvit_group.get_name())[0:6]
            for i in range(len(target)):
                target_dict[joint_name[i]] = np.deg2rad(target[i])
            rospy.logdebug("WlanSetTask.move_wait(): Planning Joint target %s", target_dict)
            self.mvit_group.set_joint_value_target(target_dict)
        elif target is geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped:
            rospy.logdebug("WlanSetTask.move_wait(): Planning Pose target %s", target)
            self.mvit_group.set_pose_target(target, end_effector_link=self.eef_link)
        else:
            rospy.logwarn("WlanSetTask.move_wait(): Illegal target type: %s", type(target))
            return
        plan_valid = False
        while not plan_valid:
            rospy.loginfo("WlanSetTask.move_wait(): Planning "+info+" to %s", self.mvit_group.get_joint_value_target())
            plan = self.mvit_group.plan()
            # Check Plan
            now = rospy.Time.now()
            rospy.sleep(5.0)
            msg = self.ui_check_cache.getElemAfterTime(now)
            if msg is None:
                plan_valid = False
            else:
                plan_valid = msg.data
            rospy.loginfo("WlanSetTask.move_wait(): Is robot allowed to execute presented plan? %s", plan_valid)
        rospy.loginfo("WlanSetTask.move_wait(): Executing ...")
        self.mvit_group.execute(plan)
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
        self.move_wait(self.waypoints["home_pose"], info="HOME")
        self.move_wait(self.waypoints["pre_grasp"], info="PreGrasp")
        # self.move_wait(self.waypoints["home_pose"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")
        # self.move_wait(self.waypoints["pre_grasp"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")
        rospy.loginfo("WlanSetTask.perform(): Opening hand ...")
        self.hand_controller.openHand()
        # self.move_wait(self.waypoints["grasp"], v=self.l_arm_speed, a=self.l_arm_acceleration, move_cmd="movel")
        self.move_wait(self.waypoints["grasp"], info="Grasp")

        rospy.loginfo("WlanSetTask.perform(): Grasp station")
        # Grasp station
        self.hand_controller.closeHand()
        rospy.sleep(5.)
        # Attach collision object to end effector
        rospy.loginfo("WlanSetTask.perform(): Attach station to end effector")
        self.mvit_scene.attach_mesh(link=self.eef_link, name=self.ssb_name, touch_links=self.touch_links)
        # self.move_wait(self.waypoints["post_grasp"], v=self.l_arm_speed, a=self.l_arm_acceleration, move_cmd="movel")
        self.move_wait(self.waypoints["post_grasp"], info="PostGrasp")

        # Set station
        rospy.loginfo("WlanSetTask.perform(): Set station")
        # Query Goal from User Interface
        ui_pose_received = False
        while not ui_pose_received:
            ui_pose = self.ui_pose_cache.getElemAfterTime(rospy.Time.now())
            ui_pose_received = ui_pose is not None
            rospy.logdebug("WlanSetTask.perform(): Set station to pose: \n %s", ui_pose)
        # Plan Path using Moveit
        self.move_wait(ui_pose)

        rospy.loginfo("WlanSetTask.perform(): Release station")
        self.hand_controller.openHand()
        rospy.sleep(5.)
        # Remove collision object from end effector
        self.mvit_scene.remove_attached_object(link=self.eef_link, name=self.ssb_name)
        # self.move_wait(self.waypoints["set_down"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")

        rospy.loginfo("WlanSetTask.perform(): Return to home pose")
        # Plan back to home station
        self.move_wait(self.waypoints["home_pose"], info="HOME")

        self.exec_thread = None

    def start(self):
        """
        Start the wlan set task
        :return: -
        :rtype: -
        """
        rospy.loginfo("WlanSetTask.start():")
        self.run_as_process(WlanSetTask.perform)


if __name__ == '__main__':
    rospy.init_node("WlanSetTask", log_level=rospy.DEBUG)
    obj = None
    for i in range(1, 4):
        if obj is None or obj.exec_thread is None:
            obj = WlanSetTask(i)
            rospy.loginfo("wlan_set_task.py: main(): Start station: %d", i)
            obj.start()
        while obj.exec_thread is not None:
            rospy.sleep(0.5)
    rospy.spin()
