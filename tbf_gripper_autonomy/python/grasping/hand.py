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
import control_msgs.msg


"""@package grasping
This package gives is made to handle a grasping task. Assuming the object of interest is within vision of a defined
camera. The position of a grasp on this object is computed by the haf_grasping package and hand to a controller. This
then manages to grasp the object by making use of MoveIt! and a HandController.
@author: Steve Grehl
"""


class HandController(object):
    """
    The HandController manages the communication with a robotiq 3 finger gripper (s-model) via an action server on a
    basic level. The complexity of the hand interface itself is shadowed in this server, so that this client can provide
    basic, straight forward functions and commands.
    Further functionality may be added in the future if needed.
    """
    def __init__(self):
        """
        Default constructor that loads parameters from the parameter server and waits for the action server to start.
        It starts with the hand "rested", meaning closed in basic mode.
        """
        server_name = rospy.get_param("~hand_server_name", "robotiqgripper_action_server")
        rospy.loginfo("hand.py@HandController(): server_name = %s", server_name)
        self.ac = actionlib.SimpleActionClient(server_name, control_msgs.msg.GripperCommandAction)
        rospy.loginfo("HandController() waiting for action server: %s  to start", server_name)
        self.hand_mode = rospy.get_param("~hand_mode", "basic")
        self.ac.wait_for_server()
        rospy.loginfo("HandController() %s  started", server_name)
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
        # http://docs.ros.org/api/control_msgs/html/msg/GripperCommand.html
        goal = control_msgs.msg.GripperCommandGoal()
        # float64 position
        # float64 max_effort

        goal.command.position = 1.22
        goal.command.max_effort = 100

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False
        rospy.sleep(3.0)

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
        # http://docs.ros.org/api/control_msgs/html/msg/GripperCommand.html
        goal = control_msgs.msg.GripperCommandGoal()
        # float64 position
        # float64 max_effort

        goal.command.position = 0.05
        goal.command.max_effort = 100

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False
        rospy.sleep(1.0)

    def restHand(self):
        """
        Closing the hand  in basic mode, so that all finger rest on the palm of the hand.
        :return: -
        :rtype: -
        """
        if self.action_pending:
            rospy.loginfo("HandController.openHand(): Action pending - abort")
            return
        # http://docs.ros.org/api/control_msgs/html/msg/GripperCommand.html
        goal = control_msgs.msg.GripperCommandGoal()
        # float64 position
        # float64 max_effort

        goal.command.position = 0.6
        goal.command.max_effort = 60

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


class DummyHandController(object):
    """
    This Dummy implementation is empty, no hand is present.
    """
    def __init__(self):
        pass

    def closeHand(self):
        pass

    def openHand(self):
        pass

    def restHand(self):
       pass

    # action server callbacks

    def done_cb(self, state, result):
        pass

    def active_cb(self, state):
        pass

    def feedback_cb(self, feedback):
        pass
