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

""" @GripperInterface
interface to the gripper using MoveIt! and the Robotiq 3 finger-gripper
consits of a finite automaton and high-level operations
@author: Steve Grehl
"""
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as transform
import math
import sys

from tbf_gripper_rqt import hand_module
from tbf_gripper_hand import GraspingMode
from GripperState import GripperState

joint_tolerances = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

class GripperInterface:
    """
    An interface to control the gripper using MoveIt and a model of the hand
    """

    def __init__(self):
        self.state = GripperState.inactive

        self.moveit_commander.roscpp_initialize(sys.argv)
        self.hand = None        # This object is an interface to the hand
        self.gripper = None     # This object is an interface to the robot as a whole.
        self.scene = None       # This object is an interface to the world surrounding the robot.
        self.ur5 = None         # This interface can be used to plan and execute motions on the arm.

        self.publisher = GripperROSHandler()    # This object publishes poses and plans using ROS


    def activate(self):
        """
        activate the gripper including hand and ur5
        :return: success
        """
        if self.state is not GripperState.inactive:
            rospy.logwarn("GripperInterface.py@GripperInterface.activate(): gripper is not inactive")
            if self.state is GripperState.error:
                rospy.loginfo("GripperInterface.py@GripperInterface.activate(): trying to recover gripper from error")
            else:
                return False
        self.state = GripperState.unknown

        # init MoveIt components - a R=S node for the ur5 takes care of the rest
        self.gripper = self.moveit_commander.RobotCommander()        # This object is an interface to the robot as a whole.
        self.scene = self.moveit_commander.PlanningSceneInterface()  # This object is an interface to the world surrounding the robot.
        self.ur5 = self.moveit_commander.MoveGroupCommander("arm")   # This interface can be used to plan and execute motions on the arm.

        # activate hand
        self.hand = hand_module.RobotiqHandModel()              # This object is an interface to the hand
        if self.hand.rACT is not 1:
            self.hand.onActivationChanged(1)
        if self.hand.rGTO is not 1:
            self.hand.onGoToChanged(1)

        self.state = GripperState.active
        return True

    def deactivate(self):
        """
        deactivate the gripper by shutting down the connection to the ur5 and Robotiq hand including dereferencing the planing, scene objects
        :return: success
        """
        if self.state is not GripperState.active:
            rospy.logwarn("GripperInterface.py@GripperInterface.deactivate(): gripper is not active")
            return False
        self.state = GripperState.error

        self.gripper = None
        self.scene = None
        self.ur5 = None

        # deactivate hand
        if self.hand.rACT is not 0:
            self.hand.onActivationChanged(0)
        self.hand = None
        self.state = GripperState.inactive

        return True

    def wait(self, sleepTime = 10.0):
        """
        wait a given amount of time
        :param sleepTime: time to sleep in seconds
        :return: None
        """
        if self.state is not GripperState.active:
            rospy.logwarn("GripperInterface.py@GripperInterface.wait(): gripper is not active")
            return False
        rospy.sleep(sleepTime)
        return True

    def move(self, goal_pose):
        """
        move the palm of the hand to the requested pose
        :param goal_pose: requested position
        :return: None
        """
        if self.state is not GripperState.active:
            rospy.logwarn("GripperInterface.py@GripperInterface.move(): gripper is not active")
            return False
        self.state = GripperState.moving
        # calculating may done outside of this function
        self.ur5.set_pose_target(goal_pose)
        plan = self.ur5.plan()

        self.publisher.publishPose(goal_pose)
        self.publisher.publishPlan(plan, startPose=self.gripper.get_current_state())

        # Moving to a pose goal
        try:
            self.ur5.go(wait=True)
        except Exception as ex:
            rospy.logerr("GripperInterface.py@GripperInterface.move(): Error during movement of the gripper, %s" % ex)
            self.state = GripperState.error
            rospy.loginfo("GripperInterface.py@GripperInterface.move(): starting error handling")
            self.errorAtMove()
            return False
        return True

    def checkPose(self, goal_pose):
        """
        check if the requested pose was reached
        :param goal_pose: requested position
        :return: None
        """
        if self.state is not GripperState.moving:
            rospy.logwarn("GripperInterface.py@GripperInterface.checkPose(): gripper is not moving")
            return False
        self.state = GripperState.unknown

        # TODO: check if the gripper is in a location to actually grasp the desired object
        # by now just assume that is the case

        self.state = GripperState.active
        return True

    def grasp(self, mode=GraspingMode.basic):
        """
        try to grasp an object with the given mode
        :param mode: grasping mode defined in GraspingMode.py@tbf_gripper_hand
        :return: success
        """
        if self.state is not GripperState.active:
            rospy.logwarn("GripperInterface.py@GripperInterface.grasp(): gripper is not active")
            return False
        self.state = GripperState.unknown
        try:
            self.hand.onMODChanged(mode)
            self.hand.mdl_fingerA.onPositionRequestChanged(255)    # fingerA is equivalent for all in the general grasping modes
        except Exception as ex:
            rospy.logerr("GripperInterface.py@GripperInterface.grasp(): Error occurred: %s" % ex)
            self.state = GripperState.error
            return False
        self.state = GripperState.grasping
        return True

    def release(self):
        """
        try to release an object from the grasp
        :return: None
        """
        if self.state is not GripperState.active:
            rospy.logwarn("GripperInterface.py@GripperInterface.release(): gripper is not active")
            return False
        self.state = GripperState.unknown
        try:
            self.hand.mdl_fingerA.onPositionRequestChanged(0)    # fingerA is equivalent for all in the general grasping modes
        except Exception as ex:
            rospy.logerr("GripperInterface.py@GripperInterface.release(): Error occurred: %s" % ex)
            self.state = GripperState.error
            return False
        self.state = GripperState.releasing
        return True

        return True

    def checkGrasp(self):
        """
        check if the assumed grasp is valid
        :return: None
        """
        if self.state is not GripperState.grasping:
            rospy.logwarn("GripperInterface.py@GripperInterface.checkGrasp(): gripper is not grasping")
            return False
        self.state = GripperState.unknown
        if self.hand.mdl_fingerA.gDT is not 2:  # contact (closing)
            rospy.logwarn("GripperInterface.py@GripperInterface.checkGrasp(): gripper did not grasp")
            return False
        elif self.hand.mdl_fingerA.gPOA > 230:  # hand is closed with no object
            rospy.logwarn("GripperInterface.py@GripperInterface.checkGrasp(): gripper did not grasp, hand is closed")
            return False
        self.state = GripperState.active
        return True

    def checkRelease(self):
        """
        check if the assumed grasp is released
        :return: None
        """
        if self.state is not GripperState.releasing:
            rospy.logwarn("GripperInterface.py@GripperInterface.checkRelease(): gripper is not releasing")
            return False
        self.state = GripperState.unknown
        if self.hand.mdl_fingerA.gDT is not 3:  # as requested
            rospy.logwarn("GripperInterface.py@GripperInterface.checkGrasp(): gripper did not release")
            return False
        elif self.hand.mdl_fingerA.gPOA > 3:  # hand is not fully open
            rospy.logwarn("GripperInterface.py@GripperInterface.checkGrasp(): gripper did not release fully")
            return False
        self.state = GripperState.active
        return True

    def errorAtMove(self):
        """
        an error occurred during the movement of the arm
        :return: success
        """
        if self.state is not GripperState.moving:
            rospy.logwarn("GripperInterface.py@GripperInterface.errorAtMove(): gripper is not moving")
            return
        self.state = GripperState.error
        return True

    def errorAtGrasp(self):
        """
        an error occurred at the grasping
        :return: success
        """
        if self.state is not GripperState.grasping:
            rospy.logwarn("GripperInterface.py@GripperInterface.errorAtGrasp(): gripper is not grasping")
            return False
        self.state = GripperState.error
        return True

    def errorAtRelease(self):
        """
        an error occurred at the releasing
        :return: success
        """
        if self.state is not GripperState.releasing:
            rospy.logwarn("GripperInterface.py@GripperInterface.errorAtGrasp(): gripper is not releasing")
            return False
        self.state = GripperState.error
        return True

    def restart(self):
        """
        try to restart the gripper from error state
        :return: success
        """
        if self.state is not GripperState.error:
            rospy.logwarn("GripperInterface.py@GripperInterface.errorAtGrasp(): gripper is not in error state")
            return False
        self.state = GripperState.inactive
        return True


class GripperROSHandler:
    """
    class to handle the communication with ROS for MoveIt!
    """
    def __init__(self):
        """
        constructor
        :return: instance of this class
        """
        ## Setup ROS
        rospy.init_node('gripper_arm', anonymous=False)
        self.trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.pose_publisher = rospy.Publisher('/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

    def publishPose(self, pose):
        """
        publish the given pose under '/target_pose'
        :param pose: pose to publish (geometry_msgs.msg.PoseStamped) via ROS
        :return: success
        """
        self.pose_publisher.publish(pose)
        return True

    def publishPlan(self, plan, startPose):
        """
        publish the given plan as trajectory under '/move_group/display_planned_path'
        :param plan: plan to publish (moveit_msgs.msg.DisplayTrajectory) via ROS
        :param startPose: start pose of the gripper (geometry_msgs.msg.PoseStamped)
        :return: success
        """
        trajectory = moveit_msgs.msg.DisplayTrajectory()
        trajectory.trajectory_start = startPose
        trajectory.trajectory.append(plan)
        self.trajectory_publisher.publish(trajectory)
        return True
