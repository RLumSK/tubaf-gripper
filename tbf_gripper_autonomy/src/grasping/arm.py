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

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rosnode

class MoveItWrapper(object):
# see: http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
    def __init__(self):
        # args2 = sys.argv

        # args2.append("_arm_prefix:=gripper_ur5_")
        # we have to load the arm_prefix from our urdf file into the namespace of an anonymous node started
        # by moveit_commander. Therefore we get all the nodes previous and after its start and select the
        # 'move_group_commander_wrappers_' and pass the parameter into its namespace.
        nodes1 = set(rosnode.get_node_names())
        moveit_commander.roscpp_initialize(sys.argv)
        nodes2 = set(rosnode.get_node_names())

        anon_node = ""
        params = rospy.get_param("/tbf_gripper_autonomy_controller/UR5")
        for n in nodes2 - nodes1:
            if n.startswith("/move_group_commander_wrappers_"):
                rospy.set_param(n+"/UR5", params)
                anon_node = n

        group_name = rospy.get_param("group_name", "UR5")  # name in the kinematics.yaml NOT SRDF
        planned_path_publisher = rospy.get_param("planned_path_publisher", "/move_group/display_planned_path")

        self.scene = moveit_commander.PlanningSceneInterface()
        self.commander = moveit_commander.RobotCommander()  # controller of the robot (arm)
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher(planned_path_publisher, moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=1)
        # RVIZ has to initilize
        rospy.sleep(1)

        self.plan = None

    def plan_to_pose(self, pose):
        self.group.set_pose_target(pose, end_effector_link="/gripper_ur5_ee_link") # TODO
        self.plan = self.group.plan()

        #display plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.commander.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        return self.plan

    def move_to_pose(self):
        self.group.go(wait=True)

#TODO
class CartesianPlaner(object):

    def __init__(self):
        pass
