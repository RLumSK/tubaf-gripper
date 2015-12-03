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

#http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

## Setup
rospy.loginfo("move_niddlefinger2pose.py: Starting setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_gripper',
                anonymous=True)

gripper = moveit_commander.RobotCommander()         # This object is an interface to the robot as a whole.
scene = moveit_commander.PlanningSceneInterface()   # This object is an interface to the world surrounding the robot.
ur5 = moveit_commander.MoveGroupCommander("arm")    # This interface can be used to plan and execute motions on the arm.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
# DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
# rospy.loginfo("move_middlefinger2pose.py: Waiting for RVIZ...")
# rospy.sleep(10)
# rospy.loginfo("move_middlefinger2pose.py: Still starting setup  ")

# ## Getting Basic Information
# rospy.loginfo("move_middlefinger2pose.py: Planing frame: %s" % ur5.get_planning_frame())
# rospy.loginfo("move_middlefinger2pose.py: End effector frame: %s" % ur5.get_end_effector_link())
# rospy.loginfo("move_middlefinger2pose.py: Robot groups:")
# rospy.loginfo(gripper.get_group_names())
# rospy.loginfo("move_middlefinger2pose.py: Robot state:")
# rospy.loginfo(gripper.get_current_state())
# input("Press Enter to continue...")

## Planning to a Pose goal
rospy.loginfo("move_middlefinger2pose.py: Generating a plan")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = -0.05
pose_target.position.z = 1.1
ur5.set_pose_target(pose_target)

plan = ur5.plan()
rospy.loginfo("move_middlefinger2pose.py: Waiting while RVIZ displays plan...")
rospy.sleep(5)

rospy.loginfo("move_middlefinger2pose.py: Visualizing plan")
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = gripper.get_current_state()
display_trajectory.trajectory.append(plan)
display_trajectory_publisher.publish(display_trajectory)

rospy.loginfo("move_middlefinger2pose.py: Waiting while plan is visualized (again)...")
rospy.sleep(5)

## Moving to a pose goal
ur5.go(wait=True)
