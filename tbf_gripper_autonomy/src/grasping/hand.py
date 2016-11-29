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

import rospy
import actionlib
import tbf_gripper_hand.msg


"""@package grasping
This package gives is made to handle a grasping task. Assuming the object of interest is within vision of a defined
camera. The position of a grasp on this object is computed by the haf_grasping package and hand to a controller. This
then manages to grasp the object by making use of MoveIt! and a HandController.
@author: Steve Grehl
"""


class HandController(object):
    """
    The HandController manages the communication with a robotiq 3 finger gripper (s-model) vi an action server on a
    basic level. The complexity of the hand interface itself is shadowed in this server, so that this client can provide
    basic, straight forward functions and commands.
    Further functionality ma be added in the future is needed.
    """
    def __init__(self):
        """
        Default constructor that loads parameters from the parameter server and waits for the action server to start.
        It starts with the hand "rested", meaning closed in basic mode.
        """
        server_name = rospy.get_param("~hand_server_name", "robotiqgripper_action_server")
        # rospy.loginfo("hand.py@HandController(): server_name = %s", server_name)
        self.ac = actionlib.SimpleActionClient(server_name, tbf_gripper_hand.msg.RobotiqGripperAction)
        rospy.loginfo("HandController() waiting for action server: %s  to start", server_name)
        self.hand_mode = rospy.get_param("~hand_mode", "basic")
        self.ac.wait_for_server()
        self.action_pending = False
        self.restHand()

    def closeHand(self):
        """
        Closing the hand by setting all its fingers to 240 or the defined maxima,
        see (robotiq/robotiq_s_model_control/config/s_model_boundaries.yaml)
        :return: -
        :rtype: -
        """
        if self.action_pending:
            rospy.loginfo("HandController.closeHand(): Action pending - abort")
            return
        goal = tbf_gripper_hand.msg.RobotiqGripperGoal()
        # goal definition
        # string mode
        # int32 position
        # int32 speed
        # int32 force
        goal.mode = self.hand_mode
        goal.position = 240
        goal.speed = 50
        goal.force = 200

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False;

    def openHand(self):
        """
        Opening the hand by setting all its fingers to 0 or the defined minima,
        see (robotiq/robotiq_s_model_control/config/s_model_boundaries.yaml)
        :return: -
        :rtype: -
        """
        if self.action_pending:
            rospy.loginfo("HandController.openHand(): Action pending - abort")
            return
        goal = tbf_gripper_hand.msg.RobotiqGripperGoal()
        # goal definition
        # string mode
        # int32 position
        # int32 speed
        # int32 force
        goal.mode = self.hand_mode
        goal.position = 0
        goal.speed = 50
        goal.force = 200

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False

    def restHand(self):
        """
        Closing the hand  in basic mode, so that all finger rest on the palm of the hand.
        :return: -
        :rtype: -
        """
        if self.action_pending:
            rospy.loginfo("HandController.openHand(): Action pending - abort")
            return
        goal = tbf_gripper_hand.msg.RobotiqGripperGoal()
        # goal definition
        # string mode
        # int32 position
        # int32 speed
        # int32 force
        goal.mode = "basic"
        goal.position = 240
        goal.speed = 100
        goal.force = 10

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False

    # action server callbacks

    def done_cb(self, state, result):
        self.action_pending = False
        pass

    def active_cb(self, state):
        pass

    def feedback_cb(self, feedback):
        pass
