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
import std_srvs.srv

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
    For further functionality see AdvancedHandController.
    """

    def __init__(self, server_name=None):
        """
        Default constructor that loads parameters from the parameter server and waits for the action server to start.
        It starts with the hand "rested", meaning closed in basic mode.
        """
        if server_name is None:
            server_name = rospy.get_param("gripper_action_server_name", "/hand/Robotiq3FGripperServer")
        rospy.loginfo("hand.py@HandController(): server_name = %s", server_name)
        self.ac = actionlib.SimpleActionClient(server_name, control_msgs.msg.GripperCommandAction)
        rospy.loginfo("HandController() waiting for action server: %s  to start", server_name)
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

        goal.command.position = 0.0
        goal.command.max_effort = 120

        self.action_pending = True
        #  goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False
        rospy.sleep(rospy.Duration(3.0))

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

        goal.command.position = 0.16  # 0.16
        goal.command.max_effort = 60

        self.action_pending = True
        # goal, done_cb = None, active_cb = None, feedback_cb = None):
        # self.ac.send_goal(goal, done_cb=self.done_cb, active_cb= self.active_cb, feedback_cb=self.feedback_cb)
        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        self.action_pending = False
        rospy.sleep(rospy.Duration(3.0))

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

        goal.command.position = 0.08
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


class AdvancedHandController(object):
    """
    The AdvancedHandController adds different modes via an action server. The complexity of the hand interface itself
    is shadowed in this server, so that this client can provide straight forward functions and commands.
    """

    def __init__(self, server_name=None):
        """
        Default constructor
        :param server_name: Basis name of the server which will be appended by '_'mode, ie _basic
        :type server_name: str
        """
        if server_name is None:
            server_name = rospy.get_param("gripper_action_server_name", "/hand/Robotiq3FGripperServer")
        self.modes = ["basic", "wide", "pinch", "scissor"]
        self.dct_controller = {}
        for s in self.modes:
            self.dct_controller[s] = HandController(server_name=server_name + "_" + s)
        self.action_pending = False

    def openHand(self, mode="basic"):
        """
        open the hand in the given mode
        :param mode: basic, wide, pinch, scissor
        :type mode: str
        :return: -
        """
        if mode not in self.dct_controller.keys():
            rospy.logwarn("AdvancedHandController.openHand(): mode %s not find candidates are: %s" % (
            mode, self.dct_controller.keys()))
            return
        if self.action_pending:
            rospy.logwarn("AdvancedHandController.openHand(): Action pending")
            return
        self.action_pending = True
        self.dct_controller[mode].openHand()
        self.action_pending = False

    def closeHand(self, mode="basic"):
        """
        close the hand in the given mode
        :param mode: basic, wide, pinch, scissor
        :type mode: str
        :return: -
        """
        if mode not in self.dct_controller.keys():
            rospy.logwarn("AdvancedHandController.closeHand(): mode %s not find candidates are: %s" % (
            mode, self.dct_controller.keys()))
            return
        if self.action_pending:
            rospy.logwarn("AdvancedHandController.closeHand(): Action pending")
            return
        self.action_pending = True
        self.dct_controller[mode].closeHand()
        self.action_pending = False

    def restHand(self, mode="basic"):
        """
        rest the hand in the given mode
        :param mode: basic, wide, pinch, scissor
        :type mode: str
        :return: -
        """
        if mode not in self.dct_controller.keys():
            rospy.logwarn("AdvancedHandController.restHand(): mode %s not find candidates are: %s" % (
            mode, self.dct_controller.keys()))
            return
        if self.action_pending:
            rospy.logwarn("AdvancedHandController.restHand(): Action pending")
            return
        self.action_pending = True
        self.dct_controller[mode].restHand()
        self.action_pending = False


class ObservativeHandController(object):
    """
    Adds the toggeling of an image service to the hand.
    If the hand gets closed the image stream should start.
    If the hand gets opened the image stream should stop.
    see also: rospkg tubaf_tools toggle_topic_by_service.py
    """

    def __init__(self, image_service=None, info_service=None, hand_controller=None):
        """
        Default constructor
        :param image_service: toggle this service to start/stop image stream
        :param hand_controller: (HandController, AdvancedHandController, DummyHandController)
        """
        if image_service is None:
            image_service = rospy.get_param("~toggle_depth_image_service_name", "")
        if info_service is None:
            info_service = rospy.get_param("~toggle_camera_info_service_name", "")
        if hand_controller is None:
            self.hand = AdvancedHandController()
        elif isinstance(hand_controller, (HandController, AdvancedHandController, DummyHandController)):
            self.hand = hand_controller
        else:
            rospy.logerr("[ObservativeHandController.__init__()] hand_controller of unsupported type %s" % type(hand_controller))
        rospy.logdebug("[ObservativeHandController.__init__()] Waiting for Image Service: %s" % image_service)
        rospy.wait_for_service(image_service)
        self.image_service = rospy.ServiceProxy(image_service, std_srvs.srv.SetBool)

        rospy.logdebug("[ObservativeHandController.__init__()] Waiting for Info Service: %s" % info_service)
        rospy.wait_for_service(info_service)
        self.info_service = rospy.ServiceProxy(image_service, std_srvs.srv.SetBool)

    def _setImageService(self, flag=True):
        """
        Set the image toggler to the given value
        :param flag: true = pass images
        :return: success
        """
        try:
            response = self.image_service(flag)
            rospy.logdebug("[ObservativeHandController._setImageService(%s)] Image Service says: %s" % (flag, response))
        except rospy.ServiceException as exc:
            rospy.logwarn("[ObservativeHandController._setImageService(%s)] Image Service did not process request: " +
                          str(exc))
        try:
            response = self.info_service(flag)
            rospy.logdebug("[ObservativeHandController._setImageService(%s)] Info Service says: %s" % (flag, response))
        except rospy.ServiceException as exc:
            rospy.logwarn("[ObservativeHandController._setImageService(%s)] Info Service did not process request: " +
                          str(exc))

    def openHand(self, mode="basic"):
        self._setImageService(False)
        if "scissor" in mode:
            return
        self.hand.openHand(mode)

    def closeHand(self, mode="basic"):
        self.hand.closeHand(mode)
        if "scissor" in mode:
            return
        self._setImageService(True)


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
