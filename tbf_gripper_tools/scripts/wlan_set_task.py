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
import moveit_commander, moveit_msgs.msg, geometry_msgs.msg, geometry_msgs.msg, std_msgs.msg
import message_filters

import autonomy.Task
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
        autonomy.Task.SetTask.__init__(self)
        self.waypoints = rospy.get_param("~waypoints_station_"+str(station_nr))
        self.exec_thread = None

        # Initialize MoveIt!
        # see: https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/scripts/move_group_python_interface_tutorial.py
        moveit_commander.roscpp_initialize(sys.argv)
        self.mvit_robot = moveit_commander.RobotCommander()
        self.mvit_scene = moveit_commander.PlanningSceneInterface()
        self.mvit_group = moveit_commander.MoveGroupCommander("UR5")
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
                                                            moveit_msgs.msg.DisplayTrajectory)
        rospy.sleep(10)

        #  Add collision object to planning scene
        self.eef_link = rospy.get_param("~eef_link", "gripper_robotiq_planning")
        ssb_default_pose = rospy.get_param("~ssb_default_pose", geometry_msgs.msg.Pose())
        ssb_mesh_filename = rospy.get_param("~ssb_mesh_filename", os.path.join(
            rospkg.RosPack().get_path('tbf_gripper_tools'), 'resources', 'mesh', 'wlan_box.stl'))
        x_scale = rospy.get_param("~ssb_x_scale", 1.0)
        y_scale = rospy.get_param("~ssb_y_scale", 1.0)
        z_scale = rospy.get_param("~ssb_z_scale", 1.0)
        self.ssb_name = "Smart_Sensor_Box"
        self.mvit_scene.add_mesh(name=self.ssb_name, pose=ssb_default_pose, filename=ssb_mesh_filename,
                                 size=(x_scale, y_scale, z_scale))

        ui_pose_topic = rospy.get_param("~ui_pose_topic", "/wlan_set_task/ui_pose")
        self.ui_pose_subscriber = message_filters.Subscriber(ui_pose_topic, geometry_msgs.msg.Pose)
        self.ui_pose_cache = message_filters.Cache(self.ui_pose_subscriber, 5)
        self.ui_check_subscriber = message_filters.Subscriber(ui_pose_topic, std_msgs.msg.Bool)
        self.ui_check_cache = message_filters.Cache(self.ui_check_subscriber, 5)

    def perform(self):
        """
        Hard coded task to pickup a wlan box from the robot, set it on the ground and the return to a home pose
        :return: -
        :rtype: -
        """
        rospy.loginfo("WlanSetTask.perform(): Move to Station on top of the Robot starting at HOME position")
        # Move to Station on top of the Robot starting at HOME position
        self.move_wait(self.waypoints["home_pose"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")
        rospy.loginfo("WlanSetTask.perform(): Opening hand ...")
        self.hand_controller.openHand()
        self.move_wait(self.waypoints["pre_grasp"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")
        self.move_wait(self.waypoints["grasp"], v=self.l_arm_speed, a=self.l_arm_acceleration, move_cmd="movel")

        rospy.loginfo("WlanSetTask.perform(): Grasp station")
        # Grasp station
        self.hand_controller.closeHand()
        rospy.sleep(5.)
        # Attach collision object to end effector
        self.mvit_scene.attach_mesh(link=self.eef_link, name=self.ssb_name)
        self.move_wait(self.waypoints["post_grasp"], v=self.l_arm_speed, a=self.l_arm_acceleration, move_cmd="movel")

        # Set station
        rospy.loginfo("WlanSetTask.perform(): Set station")
        # Query Goal from User Interface
        ui_pose = self.ui_pose_cache.getElemAfterTime(rospy.Time.now())
        # Plan Path using Moveit
        self.mvit_group.clear_pose_targets()
        self.mvit_group.set_pose_target(pose=ui_pose, end_effector_link=self.eef_link)

        plan_valid = False
        while not plan_valid:
            rospy.loginfo("WlanSetTask.perform(): Planning ...")
            plan = self.mvit_group.plan()
            # Check Plan
            now = rospy.Time.now()
            rospy.sleep(5.0)
            plan_valid = self.ui_check_cache.getElemAfterTime(now)
        rospy.loginfo("WlanSetTask.perform(): Executing ...")
        self.mvit_group.execute(plan)

        rospy.loginfo("WlanSetTask.perform(): Release station")
        self.hand_controller.openHand()
        rospy.sleep(5.)
        # Remove collision object from end effector
        self.mvit_scene.remove_attached_object(link=self.eef_link, name=self.ssb_name)
        # self.move_wait(self.waypoints["set_down"], v=self.j_arm_speed, a=self.j_arm_acceleration, move_cmd="movej")

        rospy.loginfo("WlanSetTask.perform(): Return to home pose")
        # Plan back to home station
        self.mvit_group.clear_pose_targets()
        self.mvit_group.set_joint_value_target(self.waypoints["home_pose"])

        plan_valid = False
        while not plan_valid:
            rospy.loginfo("WlanSetTask.perform(): Planning ...")
            plan = self.mvit_group.plan()
            # Check Plan
            now = rospy.Time.now()
            rospy.sleep(5.0)
            plan_valid = self.ui_check_cache.getElemAfterTime(now)
        rospy.loginfo("WlanSetTask.perform(): Executing ...")
        self.mvit_group.execute(plan)

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
