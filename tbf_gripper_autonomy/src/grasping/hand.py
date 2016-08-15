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

class HandController():

    def __init__(self):
        server_name = rospy.get_param("~hand_server_name", "robotiqgripper_action_server")
        # rospy.loginfo("hand.py@HandController(): server_name = %s", server_name)
        self.ac = actionlib.SimpleActionClient(server_name, tbf_gripper_hand.msg.RobotiqGripperAction)
        rospy.loginfo("HandController() waiting for action server: %s  to start", server_name)
        self.ac.wait_for_server()
        self.action_pending = False

    def closeHand(self):
        if self.action_pending:
            rospy.loginfo("HandController.closeHand(): Action pending - abort")
            return
        goal = tbf_gripper_hand.msg.RobotiqGripperGoal()
        # goal definition
        # string mode
        # int32 position
        # int32 speed
        # int32 force
        goal.mode = "Basic"
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
        if self.action_pending:
            rospy.loginfo("HandController.openHand(): Action pending - abort")
            return
        goal = tbf_gripper_hand.msg.RobotiqGripperGoal()
        # goal definition
        # string mode
        # int32 position
        # int32 speed
        # int32 force
        goal.mode = "Basic"
        goal.position = 0
        goal.speed = 50
        goal.force = 200

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False;

    def restHand(self):
        if self.action_pending:
            rospy.loginfo("HandController.openHand(): Action pending - abort")
            return
        goal = tbf_gripper_hand.msg.RobotiqGripperGoal()
        # goal definition
        # string mode
        # int32 position
        # int32 speed
        # int32 force
        goal.mode = "Basic"
        goal.position = 100
        goal.speed = 50
        goal.force = 0

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False;

    # action server callbacks

    def done_cb(self, state, result):
        self.action_pending = False
        pass

    def active_cb(self, state):
        pass

    def feedback_cb(self, feedback):
        pass
