#!/usr/bin/python
# coding=utf-8
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

import random
import signal
import sys

import rospy
import time
import os
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from tbf_gripper_rqt.hand_module import RobotiqHandModel
from tbf_gripper_tools.DemoStatus import *

import numpy as np

import thread

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
UP_JS = [0.0, -45.0, -95, -150, -90, -39]
LOW_JS = [0.0, -15.0, -115, -57, -90, -45]
HOME_POS = [0.0, -90, 0, -90, 0, 0]


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


HOME_PROGRAM = \
    """movej(%(homepos)s, a=%(a)f, v=%(v)f)
    movej(%(lowpos)s, a=%(a)f, v=%(v)f)
    """ % {'homepos': pos2str(HOME_POS), 'lowpos': pos2str(LOW_JS),
       'a': np.deg2rad(20), 'v': np.deg2rad(45)}


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class SchnickSchnackSchnuckHandModel(RobotiqHandModel):
    """
    model for the Schnick Schnack Schnuck demo, handels the different hand poses
    """

    def __init__(self):
        """
        default constructor - set parameter, starting hand
        """
        super(SchnickSchnackSchnuckHandModel, self).__init__()

        # initilize Hand
        self.onActivationChanged(1)

        # set Speed and Force
        force = 200
        speed = 200
        self.mdl_fingerA.onForceRequestChanged(force)
        self.mdl_fingerA.onSpeedRequestChanged(speed)
        self.mdl_fingerB.onForceRequestChanged(force)
        self.mdl_fingerB.onSpeedRequestChanged(speed)
        self.mdl_fingerC.onForceRequestChanged(force)
        self.mdl_fingerC.onSpeedRequestChanged(speed)
        self.mdl_fingerS.onForceRequestChanged(force)
        self.mdl_fingerS.onSpeedRequestChanged(speed)

        # random stuff
        random.seed()

        # set finger/scissor control
        self.onIndividualFingerChanged(1)
        self.onIndividualScissorChanged(1)

        # start movement
        self.onGoToChanged(1)

        # result publisher
        result_topic = rospy.get_param("result_topic", "/schnick_result")
        self.result_publisher = rospy.Publisher(result_topic, String, queue_size=10)

    def setStone(self):
        """
        sets the equivalent of a stone form (all fingers closed)
        :return: -
        :rtype: None
        """
        self.mdl_fingerA.onPositionRequestChanged(200)
        self.mdl_fingerB.onPositionRequestChanged(200)
        self.mdl_fingerC.onPositionRequestChanged(200)
        self.mdl_fingerS.onPositionRequestChanged(0)
        rospy.loginfo("SchnickSchnackSchnuckModel.setStone")

    def setScisscor(self):
        """
        sets the equivalent of a sciccor pose (middle finger closed, adjacent fingers spread)
        :return: -
        :rtype: None
        """
        self.mdl_fingerA.onPositionRequestChanged(200)
        self.mdl_fingerB.onPositionRequestChanged(0)
        self.mdl_fingerC.onPositionRequestChanged(0)
        self.mdl_fingerS.onPositionRequestChanged(0)
        rospy.loginfo("SchnickSchnackSchnuckModel.setScissor")

    def setPaper(self):
        """
        sets the equivalent of a paper pose (all fingers open, but sciccor closed)
        :return: -
        :rtype: None
        """
        self.mdl_fingerA.onPositionRequestChanged(0)
        self.mdl_fingerB.onPositionRequestChanged(0)
        self.mdl_fingerC.onPositionRequestChanged(0)
        self.mdl_fingerS.onPositionRequestChanged(200)
        rospy.loginfo("SchnickSchnackSchnuckModel.setPaper")

    def setFountain(self):
        """
        sets the equivalent of a fountain pose (all fingers in the middle)
        :return: -
        :rtype: None
        """
        self.mdl_fingerA.onPositionRequestChanged(120)
        self.mdl_fingerB.onPositionRequestChanged(120)
        self.mdl_fingerC.onPositionRequestChanged(120)
        self.mdl_fingerS.onPositionRequestChanged(82)
        rospy.loginfo("SchnickSchnackSchnuckModel.setFountain")

    def setPose(self, publish=True):
        """
        randomly set a hand pose and publish the result
        :param publish: publish the result via ROS or not
        :type publish: bool
        :return: -
        :rtype: None
        """
        decision = random.random()
        result = ""
        if decision > 0.4:
            if decision > 0.7:
                self.setStone()
                result = "stone"
            else:
                self.setScisscor()
                result = "scissors"
        else:
            if decision > 0.1:
                self.setPaper()
                result = "paper"
            else:
                self.setFountain()
                result = "fountain"
        self.sendROSMessage()
        if publish:
            self.result_publisher.publish(result)
            # rospy.sleep(3.)
        return result


class SchnickSchnackSchnuckController():
    def __init__(self):
        # ROS Anbindung
        self.sub = rospy.Subscriber("/schnick_cmd", String, self.execute, queue_size=1)

        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.onJs, queue_size=1)

        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)
        self.lasttime = time.time()
        self.last_start = time.time()

        image_dir = rospy.get_param("~image_dir", None)
        camera_topic = rospy.get_param("~cam_topic", None)

        self.result_img_msgs = dict()

        if image_dir:
            br = CvBridge()
            imgs = ("fountain", "paper", "scissors", "stone")
            for i in imgs:
                img_path = os.path.join(image_dir, i + ".jpg")
                if os.path.isfile(img_path):
                    cv_img = cv2.imread(img_path,cv2.CV_LOAD_IMAGE_UNCHANGED)
                    self.result_img_msgs[i] = br.cv2_to_imgmsg(cv_img,encoding="passthrough")
                    rospy.loginfo("loaded resultimage %s",img_path)
            self.result_img_pub = rospy.Publisher("schnick_result_img",Image,queue_size=1)
        else:
            self.result_img_pub = None

        self.pub_img = False
        if camera_topic:
            self.cam_pub = rospy.Publisher("schnick_cam_img",Image,queue_size=1)
            self.cam_sub = rospy.Subscriber(camera_topic,Image,callback=self.on_camImage)

        self.demo_monitor = DemoStatus("schnick")
        self.demo_monitor.set_status(DemoState.stop)

        # Hand
        self.hand = SchnickSchnackSchnuckHandModel()

        self.move_dur = 1.0
        self.cur_pos = np.zeros((6,))
        self.exec_thread = None
        rospy.sleep(0.5)

        if rospy.get_param("~no_init", False):
            rospy.logwarn("init skipped!")
            self.demo_monitor.set_status(DemoState.error)
        else:
            self.run_as_process(SchnickSchnackSchnuckController.initialise)

            # Arm

    def on_camImage(self,img):
        if self.pub_img:
            self.pub_img = False
            rospy.loginfo("republish camera image")
            self.cam_pub.publish(img)

    def pub_result(self,res_str):
        img = self.result_img_msgs.get(res_str)
        self.pub_img = True
        if img:
            rospy.loginfo("pub result %s",res_str)
            self.result_img_pub.publish(img)
        else:
            rospy.loginfo("no image loaded for %s",res_str)

    def initialise(self):
        rospy.sleep(2.)
        self.hand.setPose(publish=False)
        rospy.sleep(0.5)
        self.moveWait(HOME_POS, v=45, a=20)
        self.moveWait(LOW_JS, v=45, a=20)
        rospy.sleep(0.5)
        self.demo_monitor.set_status(DemoState.initialized)
        self.exec_thread = None

    def run_as_process(self, function):
        """
        @param function: the function to start
        @param args: tuple of function args
        """
        # p = Process(target=function, args=(self,))
        # p.start()
        # self.exec_thread = p
        self.exec_thread = thread.start_new_thread(function, (self,))

    def onJs(self, js):
        if time.time() - self.lasttime > 0.02:
            pp = list(js.position)
            pp[0] = js.position[2]
            pp[2] = js.position[0]

            self.cur_pos = np.rad2deg(pp)
            self.lasttime = time.time()

    def moveWait(self, pose, goal_tolerance=0.5, v=None, a=None, t=None):
        prog = "movej(%s" % pos2str(pose)
        if a is not None:
            prog += ", a=%f" % np.deg2rad(a)
        if v is not None:
            prog += ", v=%f" % np.deg2rad(v)
        if t is not None:
            prog += ", t=%f" % t
        prog += ")"
        if self.exec_thread is not None:
            self.program_pub.publish(prog)
        else:
            raise InterruptError("Thread interrupted")
        while True:
            dst = np.abs(np.subtract(pose, self.cur_pos))
            if np.max(dst) < goal_tolerance:
                break
            rospy.sleep(0.02)

    def perform_demo(self):
        self.demo_monitor.set_status(DemoState.running)
        self.hand.setPose(publish=False)
        final_pose = ""
        for i in range(3):
            if i == 2:
                final_pose = self.hand.setPose(publish=True)
            self.moveWait(UP_JS, t=0.5)
            self.moveWait(LOW_JS, t=0.5)

        rospy.sleep(1.5)
        self.pub_result(final_pose)
        self.demo_monitor.set_status(DemoState.pause)
        self.exec_thread = None

    def execute(self, msg):
        if msg.data.lower().startswith("stop"):
            self.program_pub.publish("stopj(6.3)")
            if self.exec_thread is not None:
                self.exec_thread = None
            rospy.logwarn("stop received!, aborting trajectory execution!")
            self.demo_monitor.set_status(DemoState.error)
            return
        if self.demo_monitor.get_status() in (DemoState.error, DemoState.unknown):
            if msg.data.startswith("start"):
                if time.time() - self.last_start < 2.0 and self.exec_thread is None:
                    rospy.loginfo("got start twice within 2 secs, reinitialising...")
                    self.run_as_process(SchnickSchnackSchnuckController.initialise)
                    return
                self.last_start = time.time()
                self.demo_monitor.set_status(DemoState.unknown)
                rospy.loginfo(
                    "error state, start received...   Press start again within 2 secs to perform reinitialisation")
            return

        if msg.data.startswith("start"):
            if self.exec_thread is None:
                self.run_as_process(SchnickSchnackSchnuckController.perform_demo)
                rospy.loginfo("starting demo...")
                return
            else:
                rospy.logwarn("demo is already running, ignoring start command!")

        rospy.loginfo("Not executing SchnickSchnackSchnuck demo - received:" + msg.data)


def test_SchnickSchnackSchnuckHandModel():
    print("Hello world")
    # Test Hand Poses
    rospy.init_node("Test_SchnickSchnackSchnuckHandModel")
    obj = SchnickSchnackSchnuckHandModel()
    obj.setPaper()
    obj.setScisscor()
    obj.setStone()
    obj.setFountain()
    # For Testing
    while True:
        obj.setPose()
        rospy.sleep(2.0)


def test_SchnickSchnackSchnuckController():
    print("Hello world")
    # Test Hand Poses
    rospy.init_node("Test_SchnickSchnackSchnuckController")
    obj = SchnickSchnackSchnuckController()
    rospy.spin()


if __name__ == '__main__':
    # test_SchnickSchnackSchnuckHandModel()
    test_SchnickSchnackSchnuckController()
