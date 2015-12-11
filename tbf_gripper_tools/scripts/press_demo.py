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

"""
press demo at 16.12.2015
@author: Steve Grehl
"""

import sys
import copy
import math
import numpy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf.transformations as transform
import tbf_gripper_rqt.gripper_module as gm

# joint_init_values = {'gripper_ur5_base_link-base_fixed_joint': 0, 'gripper_ur5_shoulder_pan_joint': 0, 'gripper_ur5_shoulder_lift_joint': -numpy.pi/4, 'gripper_ur5_elbow_joint': 0, 'gripper_ur5_wrist_1_joint': -numpy.pi/4, 'gripper_ur5_wrist_2_joint': 0, 'gripper_ur5_wrist_3_joint': 0, 'gripper_ur5_ee_fixed_joint': 0, 'gripper_ur5_wrist_3_link-tool0_fixed_joint': 0}
joint_init_values = {'gripper_ur5_shoulder_pan_joint': -numpy.pi, 'gripper_ur5_shoulder_lift_joint': -numpy.pi/4,
                     'gripper_ur5_elbow_joint': 0, 'gripper_ur5_wrist_1_joint': -numpy.pi/4,
                     'gripper_ur5_wrist_2_joint': 0, 'gripper_ur5_wrist_3_joint': 0}
mobile_joint_names = ['gripper_ur5_shoulder_pan_joint', 'gripper_ur5_shoulder_lift_joint', 'gripper_ur5_elbow_joint',
                      'gripper_ur5_wrist_1_joint', 'gripper_ur5_wrist_2_joint', 'gripper_ur5_wrist_3_joint']


class Demo(object):
    """
    Demo for the picking and placing a wlan station using MoveIt
    """

    def __init__(self):
        """
        Default constructor: initilize ROS, Listener, Subscriber, connect to the hardware
        :return: object
        """
        rospy.loginfo("press_Demo.py: Starting init")

        # Setup ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('press_demo', anonymous=False)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.display_target_pose_publisher = rospy.Publisher('/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.obj_recognition_command_publisher = rospy.Publisher('/obj_cmd', std_msgs.msg.Int8, queue_size=10)
        self.emergency_stop_subscriber = rospy.Subscriber("/emergency_stop", std_msgs.msg.Bool, self.onEmergencyStop)
        self.box_pose_subscriber = None

        #  Setup robot controller
        self.hand = gm.WideGripperModel()
        self.gripper = moveit_commander.RobotCommander()         # This object is an interface to the robot as a whole.
        self.scene = moveit_commander.PlanningSceneInterface()   # This object is an interface to the world surrounding the robot.
        self.ur5 = moveit_commander.MoveGroupCommander("UR5")    # This interface can be used to plan and execute motions on the arm.
        self.moveit_hand = moveit_commander.MoveGroupCommander("Hand")
        self.ur5.allow_replanning(True)
        self.ur5.set_planning_time(30)
        self.ur5.set_goal_tolerance(0.01)
        self.ur5.set_workspace([-2, -2, -2, 2, 2, 2])

        self.grasp_box = False
        self.isStopped = False

        self.cmd_obj = std_msgs.msg.Int8()
        self.obj_box_pose = None

    def _go(self, plan=None, comander=None):
        if (True == self.isStopped):
            rospy.logwarn("press_demo.py@Demo._go: UR5 Stopped (%s)" % self.isStopped)
            return False
        print comander
        if comander is None:
            comander = self.ur5
        comander.set_start_state_to_current_state()
        if plan is None:
            plan = self.ur5.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        if (False == bool(input("Move UR5? (True/False):"))):
            return False
        rospy.loginfo("press_demo.py@Demo._go: Start moving")
        ## Moving to a pose goal
        comander.go(wait=True)
        rospy.loginfo("press_demo.py@Demo._go: Stopped moving?")
        return True

    def _move_joints(self, dct_joints):
        self.ur5.set_joint_value_target(dct_joints)
        return self._go()

    def onNewBoxPose(self, msg):
        """
        Callback function for the box detection, called after the box was found and a approx. pose was estimated
        :param msg: PoseStamped msg with the box pose
        :return: None
        """
        if(False == self.grasp_box):
            return
        pose = msg.pose

        # rotate to encounter the marker from the opposite position
        # http://answers.ros.org/question/69754/quaternion-transformations-in-python/
        # quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        # euler = transform.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]
        # # roll = roll + math.pi/2.0
        # # pitch = pitch + math.pi/2.0
        # yaw = yaw - math.pi/2.0
        #
        # quaternion = transform.quaternion_from_euler(roll, pitch, yaw)
        targetPose = pose
        # targetPose.pose.orientation.x = quaternion[0]
        # targetPose.pose.orientation.y = quaternion[1]
        # targetPose.pose.orientation.z = quaternion[2]
        # targetPose.pose.orientation.w = quaternion[3]

        self.obj_box_pose = targetPose
        self.box_pose_subscriber.unregister()
        self.grasp_box = True
        self.cmd_obj.data = -1
        self.obj_recognition_command_publisher.publish(self.cmd_obj)

    def onEmergencyStop(self, msg):
        if msg.data:
            self.ur5.stop()
            self.isStopped = True
            rospy.logwarn("Stopped UR5 due to an emergency stop command")

    def move2start(self):
        """
        Move in to the Home position of the UR5, arm is stretched upwards
        :return: None
        """
        #http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
        # Planning to a joint-space goal
        self.ur5.clear_pose_targets()
        self.ur5.set_named_target("Home-UR5")
        plan = self.ur5.plan()
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory.append(plan)
        # self.display_trajectory_publisher.publish(display_trajectory)
        self._go(plan)
        # ['gripper_ur5_shoulder_pan_joint', 'gripper_ur5_shoulder_lift_joint', 'gripper_ur5_elbow_joint',
        # 'gripper_ur5_wrist_1_joint', 'gripper_ur5_wrist_2_joint', 'gripper_ur5_wrist_3_joint']
        # joints = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
        # self._move_joints(joints)

    def moveBase(self, rad_base_angular):
        self.ur5.clear_pose_targets()
        self._move_joints({mobile_joint_names[0]: (joint_init_values[mobile_joint_names[0]] + rad_base_angular)})

    def moveElbow(self, rad_elbow_angular):
        self.ur5.clear_pose_targets()
        self._move_joints({mobile_joint_names[2]: (-0.90757121 - rad_elbow_angular)})

    def adjust(self):
        self.ur5.clear_pose_targets()
        joints = {mobile_joint_names[0]: -numpy.pi,    # Base
                  mobile_joint_names[1]: -numpy.pi/2,  # Shoulder
                  mobile_joint_names[2]: -0.90757121,  # Elbow
                  mobile_joint_names[3]: -4.22369679,  # Wrist 1
                  mobile_joint_names[4]: -1.6406095,   # Wrist 2
                  mobile_joint_names[5]: -0.89011792}  # Wrist 3
        print joints
        self._move_joints(joints)

    def planFinalMove(self):
        self.ur5.clear_pose_targets()
        self.moveit_hand.clear_pose_targets()
        self.moveit_hand.set_pose_target(self.obj_box_pose)
        return self.moveit_hand.plan()

    def demo_pick(self):
        rospy.loginfo("Press Demo 16/12/2015 - Find the WLAN Box and grap it!")
        # # Planning to a Pose goal
        # rospy.loginfo("press_demo.py: Phase  1/10: -Move to Starting Position-")
        # self.move2start()
        # self.hand.closeGripper()
        # rospy.loginfo("press_demo.py: Phase  2/10: -Rotate for Lighthouse Model-")
        # for i in range(1, 9):
        #     self.moveBase(i*numpy.pi/4)
        # rospy.loginfo("press_demo.py: Phase  3/10: -Rotate to Starting Position and adjust Wrist Joints-")
        self.move2start()
        self.adjust()
        rospy.loginfo("press_demo.py: Phase  4/10: -Rotate Elbow for Front Model || Object Recognition -")
        #TODO start object recognition MC
        self.cmd_obj.data = 0  # init object recognition
        self.box_pose_subscriber = rospy.Subscriber("/obj_pose", geometry_msgs.msg.PoseStamped, self.onNewBoxPose)
        self.obj_recognition_command_publisher.publish(self.cmd_obj)
        self.grasp_box = True
        for i in range(1, 3):
            if self.obj_box_pose is not None:
                break
            self.moveElbow(i*numpy.pi/8)
        self.cmd_obj.data = 1  # finish object recognition and publish /obj_pose
        self.obj_recognition_command_publisher.publish(self.cmd_obj)
        while not self.grasp_box:
            rospy.loginfo("press_demo.py: Waiting for Target Pose")
            rospy.sleep(1)
        rospy.loginfo("press_demo.py: Phase  7/10: -Open the 3F Hand-")
        self.hand.openGripper()
        rospy.loginfo("press_demo.py: Phase  6/10: -Pose Planning-")
        final_plan = self.planFinalMove()
        rospy.loginfo("press_demo.py: Phase  8/10: -Move to Planned Pose-")
        self._go(comander=self.moveit_hand, plan=final_plan)
        rospy.loginfo("press_demo.py: Phase  9/10: -Grasp-")
        self.hand.closeGripper()
        rospy.loginfo("press_demo.py: Phase 10/10: -Move to Starting Position-")
        self.move2start()

if __name__=='__main__':
    print "I'm alive!\n"
    try:
        rospy.loginfo("ROS is alive")
        new_demo = Demo()
        new_demo.demo_pick()
        #time.sleep(10)
        #new_demo.demo_place()
    except rospy.ROSInterruptException:
        new_demo.ur5.stop()
        pass
