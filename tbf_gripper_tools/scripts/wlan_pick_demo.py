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
import time
import numpy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Quaternion, Point
import std_msgs.msg
import tf.transformations as transform
import tbf_gripper_rqt.gripper_module as gm

# joint_init_values = {'gripper_ur5_base_link-base_fixed_joint': 0, 'gripper_ur5_shoulder_pan_joint': 0, 'gripper_ur5_shoulder_lift_joint': -numpy.pi/4, 'gripper_ur5_elbow_joint': 0, 'gripper_ur5_wrist_1_joint': -numpy.pi/4, 'gripper_ur5_wrist_2_joint': 0, 'gripper_ur5_wrist_3_joint': 0, 'gripper_ur5_ee_fixed_joint': 0, 'gripper_ur5_wrist_3_link-tool0_fixed_joint': 0}
joint_init_values = {'gripper_ur5_shoulder_pan_joint': -numpy.pi, 'gripper_ur5_shoulder_lift_joint': -numpy.pi/4,
                     'gripper_ur5_elbow_joint': 0, 'gripper_ur5_wrist_1_joint': -numpy.pi/4,
                     'gripper_ur5_wrist_2_joint': 0, 'gripper_ur5_wrist_3_joint': 0}
mobile_joint_names = ['gripper_ur5_shoulder_pan_joint', 'gripper_ur5_shoulder_lift_joint', 'gripper_ur5_elbow_joint',
                      'gripper_ur5_wrist_1_joint', 'gripper_ur5_wrist_2_joint', 'gripper_ur5_wrist_3_joint']

GRIPPER_PLAN_OFFSET = 0.2

sample_pose = Pose(Point(-0.843,0.166350559267,-0.095),
                   Quaternion(-0.338466495763,0.60137533094,0.337815120851,0.640053971714))
# sample_pose = Pose(Point(-0.833062205145,0.166350559267,-0.114325897108),
#                    Quaternion(-0.338466495763,0.60137533094,0.337815120851,0.640053971714))

class Demo(object):
    """
    Demo for the picking and placing a wlan station using MoveIt
    """

    def __init__(self):
        """
        Default constructor: initilize ROS, Listener, Subscriber, connect to the hardware
        :return: object
        """

        # Setup ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('wlan_pick_demo', anonymous=False)
        rospy.loginfo("wlan_pick_demo.py: Starting init")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.display_target_pose_publisher = rospy.Publisher('/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.obj_recognition_command_publisher = rospy.Publisher('/obj_cmd', std_msgs.msg.Int8, queue_size=10)
        self.emergency_stop_subscriber = rospy.Subscriber("/emergency_stop", std_msgs.msg.Bool, self.onEmergencyStop)
        self.box_pose_subscriber = None

        #  Setup robot controller
        self.hand = gm.WideGripperModel()
        self.gripper = None
        self.scene = moveit_commander.PlanningSceneInterface()   # This object is an interface to the world surrounding the robot.
        self.ur5 = moveit_commander.MoveGroupCommander("UR5")    # This interface can be used to plan and execute motions on the arm.

        self.ur5.allow_replanning(False)
        self.ur5.set_planning_time(10)
        self.ur5.set_goal_position_tolerance(1)
        self.ur5.set_goal_orientation_tolerance(1)
        size = 8.0
        self.ur5.set_workspace([-size/2.0, -size/2.0, -size/2.0, size/2.0, size/2.0, size/2.0])  #TODO

        self.manipulator = None

        self.grasp_box = False
        self.isStopped = False

        self.cmd_obj = std_msgs.msg.Int8()
        self.pre_grasp_pose = None
        self.wlan_box_size = (0.17, 0.14, 0.1)

        #TODO start DO0 at the UR5 and initilize the hand

    def _init_manipulator(self):
        print "_init_manipulator"
        self.manipulator = moveit_commander.MoveGroupCommander("manipulator")
        self.manipulator.allow_replanning(True)
        self.manipulator.set_planning_time(10)
        self.manipulator.set_goal_tolerance(0.1)
        self.manipulator.set_workspace([-2, -2, -2, 2, 2, 2])

    def _go(self, plan=None, comander=None):
        print "_go"
        if (True == self.isStopped):
            rospy.logwarn("wlan_pick_demo.py@Demo._go: UR5 Stopped (%s)" % self.isStopped)
            return False
        if comander is None:
            comander = self.ur5
        comander.set_start_state_to_current_state()
        if plan is None:
            # THIS TAKES TO MUCH TIME 12/12/15
            plan = self.ur5.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        if (False == bool(input("Move UR5? (True/False):"))):
            return False
        rospy.loginfo("wlan_pick_demo.py@Demo._go: Start moving")
        # Moving to a pose goal
        # comander.go(wait=True)
        comander.execute(plan)
        rospy.loginfo("wlan_pick_demo.py@Demo._go: Stopped moving?")
        comander.clear_pose_targets()
        print "_go - end"
        return True

    def _move_joints(self, dct_joints):
        print "_move_joints"
        self.ur5.set_joint_value_target(dct_joints)
        return self._go()

    def onNewBoxPose(self, msg):
        """
        Callback function for the box detection, called after the box was found and a approx. pose was estimated
        :param msg: PoseStamped msg with the box pose
        :return: None
        """
        print "onNewBoxPose"
        # WLAN Box found
        # add a box to the planning_scene - occupancy map should remove those voxel
        # see: http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html
        box_pose = msg.pose
        rospy.loginfo("@onNewBoxPose: Adding the wlna_box into the world at the location of the given pose:%s" % box_pose.position)
        self.scene.add_box("wlan_box", box_pose, size=self.wlan_box_size)

        self.pre_grasp_pose = box_pose
        self.pre_grasp_pose.position.x += GRIPPER_PLAN_OFFSET

        self.box_pose_subscriber.unregister()
        self.grasp_box = True
        self.cmd_obj.data = -1
        self.obj_recognition_command_publisher.publish(self.cmd_obj)

    def onEmergencyStop(self, msg):
        print "onEmergencyStop"
        if msg.data:
            self.ur5.stop()
            self.isStopped = True
            rospy.logwarn("Stopped UR5 due to an emergency stop command")

    def move2start(self):
        """
        Move in to the Home position of the UR5, arm is stretched upwards
        :return: None
        """
        print "move2start"
        #http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
        # Planning to a joint-space goal
        self.ur5.set_named_target("Home-UR5")
        self._go()

    def moveBase(self, rad_base_angular):
        print "moveBase"
        self._move_joints({mobile_joint_names[0]: (joint_init_values[mobile_joint_names[0]] + rad_base_angular)})

    def moveElbow(self, rad_elbow_angular):
        print "moveElbow"
        self._move_joints({mobile_joint_names[2]: (-0.90757121 - rad_elbow_angular)})

    def adjust(self):
        print "adjust"
        joints = {mobile_joint_names[0]: -numpy.pi,    # Base
                  mobile_joint_names[1]: -numpy.pi/2,  # Shoulder
                  mobile_joint_names[2]: -0.90757121,  # Elbow
                  mobile_joint_names[3]:  2.0594885,  # Wrist 1
                  mobile_joint_names[4]: -1.6406095,   # Wrist 2
                  mobile_joint_names[5]: -0.89011792}  # Wrist 3
        self._move_joints(joints)

    def plan_grasp_move(self):
        print "plan_grasp_move"
        self.ur5.clear_pose_targets()
        self.manipulator.clear_pose_targets()
        # self.manipulator.set_pose_target(sample_pose)
        self.manipulator.set_pose_target(self.obj_box_pose)
        return self.manipulator.plan()

    def grasp(self):
        print "grasp"
        # let hand grasp and wait to succeed
        self.hand.closeGripper() #TODO <-- change to wait till goal is reached or throw exception
        
        # add wlan_box object to the robot
        self.scene.remove_world_object("wlan_box")
        # link, name, pose, size = (1, 1, 1), touch_links = []
        # gripper_ur5_finger_1_link_0
        self.gripper = moveit_commander.RobotCommander()         # This object is an interface to the robot as a whole.
        finger_links = [self.gripper.get_link("gripper_robotiq_finger_1_link_0"),
                        self.gripper.get_link("gripper_robotiq_finger_1_link_1"),
                        self.gripper.get_link("gripper_robotiq_finger_1_link_2"),
                        self.gripper.get_link("gripper_robotiq_finger_1_link_3"),
                        self.gripper.get_link("gripper_robotiq_finger_2_link_0"),
                        self.gripper.get_link("gripper_robotiq_finger_2_link_1"),
                        self.gripper.get_link("gripper_robotiq_finger_2_link_2"),
                        self.gripper.get_link("gripper_robotiq_finger_2_link_3"),
                        self.gripper.get_link("gripper_robotiq_finger_middle_link_0"),
                        self.gripper.get_link("gripper_robotiq_finger_middle_link_1"),
                        self.gripper.get_link("gripper_robotiq_finger_middle_link_2"),
                        self.gripper.get_link("gripper_robotiq_finger_middle_link_3")
                        ]
        self.scene.attach_box(self.gripper.get_link("gripper_robotiq_palm"), "wlan_box", self.wlan_box_size, touch_links=finger_links)


    def demo_pick(self):
        rospy.loginfo("Press Demo 16/12/2015 - Find the WLAN Box and grap it!")
        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick.py: Phase  1/10: -Move to Starting Position-")
        # Move to initial position
        self.move2start()
        self.hand.closeGripper() #TODO <-- change to wait till goal is reached or throw exception

        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick.py: Phase  2/10: -Rotate for Lighthouse Model-")
        # Rotate Base joint to get a 360deg scene for the occupancy map
        for i in range(0, 6):
            self.moveBase(i*numpy.pi/3)

        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick.py: Phase  3/10: -Rotate to Starting Position and adjust Wrist Joints-")
        # Move to initial position for the object recognition step - find the WLAN box now
        self.move2start()
        self.adjust()

        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick.py: Phase  4/10: -Rotate Elbow for Front Model || Object Recognition -")
        # Init object recognition
        self.cmd_obj.data = 0  # init object recognition
        self.box_pose_subscriber = rospy.Subscriber("/obj_pose", geometry_msgs.msg.PoseStamped, self.onNewBoxPose)
        self.obj_recognition_command_publisher.publish(self.cmd_obj)

        # Move ur5 to find the WLAN box
        for i in range(1, 3):
            if self.pre_grasp_pose is not None:
                break
            self.moveElbow(i*numpy.pi/8)
        rospy.sleep(3)
        # Stop object recognition
        self.cmd_obj.data = 1  # finish object recognition and publish /obj_pose
        self.obj_recognition_command_publisher.publish(self.cmd_obj)

        while not self.grasp_box:
            rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick: Waiting for Target Pose")
            rospy.sleep(1)


        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick: Phase  5/10: -Open the 3F Hand-")
        self.hand.openGripper()  #TODO <-- change to wait till goal is reached or throw exception
        
        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick: Phase  6/10: -Plan Grasping Position-")
        final_plan = self.plan_grasp_move()
        
        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick: Phase  8/10: -Move to Grasping Position-")
        self._init_manipulator()
        self._go(comander=self.manipulator, plan=final_plan)
        
        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick: Phase  9/10: -Grasp-")
        self.grasp()

        rospy.loginfo("tbf_gripper_tools/scripts/wlan_pick_demo.py@demo_pick: Phase 10/10: -Move to Starting Position-")
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
