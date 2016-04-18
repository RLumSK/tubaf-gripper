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
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from tbf_gripper_rqt.hand_module import RobotiqHandModel
from tbf_gripper_tools.DemoStatus import *


import numpy as np

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
UP_JS = [0.0, -45.0, -95, -150, -90, -39]
LOW_JS = [0.0, -15.0, -115, -50, -90, -45]
HOME_POS = [0.0, -90, 0, -90, 0, 0]


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

    def setPose(self):
        """
        randomly set a hand pose and publish the result
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
        self.result_publisher.publish(result)
        # rospy.sleep(3.)


class SchnickSchnackSchnuckController():
    def __init__(self):
        # ROS Anbindung
        self.sub = rospy.Subscriber("/schnick_cmd", String, self.execute, queue_size=1)

        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.onJs, queue_size=1)

        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)
        self.isExecuting = True
        self.lasttime = time.time()

        self.demo_monitor = DemoStatus("schnick")
        self.demo_monitor.set_status(DemoState.stop)

        # Hand
        self.hand = SchnickSchnackSchnuckHandModel()
        rospy.sleep(2.)
        self.hand.setPose()

        self.move_dur = 1.0
        self.cur_pos = np.zeros((6,))

        # Arm
        # self.ur5_mover = TrajectoryExecutor.TrajectoryExecutor( '/ur5/pos_based_pos_traj_controller/follow_joint_trajectory',
        #                                                         ="gripper_ur5_", state_topic="/ur5/joint_states")
        # self.ur5_mover.move_home()
        # self.move_down(5.0)

        self.isExecuting = False

        rospy.sleep(0.5)
        prg = HOME_PROGRAM.replace("\n","\t")
        # for line in HOME_PROGRAM.strip().split('\n'):
        #     self.program_pub.publish(line)
        #     rospy.sleep(3.5)

        self.moveWait(HOME_POS,v=45,a=20)
        self.moveWait(LOW_JS,v=45,a=20)

        rospy.sleep(0.5)
        self.demo_monitor.set_status(DemoState.initialized)

    def onJs(self,js):
        if time.time() - self.lasttime > 0.02:
            pp = list(js.position)
            pp[0] = js.position[2]
            pp[2] = js.position[0]

            self.cur_pos = np.rad2deg(pp)
            self.lasttime = time.time()

    def moveWait(self,pose,goal_tolerance = 0.5,v=None,a=None,t=None):
        prog = "movej(%s" % pos2str(pose)
        if a is not None:
            prog += ", a=%f" % np.deg2rad(a)
        if v is not None:
            prog += ", v=%f" % np.deg2rad(v)
        if t is not None:
            prog += ", t=%f" % t
        prog += ")"
        self.program_pub.publish(prog)
        while True:
            dst = np.abs(np.subtract(pose,self.cur_pos))
            if np.max(dst) < goal_tolerance:
                break
            rospy.sleep(0.02)

    def execute(self, msg):
        if self.isExecuting:
            rospy.loginfo("Not executing Schnick,Schnack,Schnuck Demo - action pending")
            return
        if not msg.data.startswith("start"):
            rospy.loginfo("Not executing Schnick,Schnack,Schnuck Demo - received:" + msg.data)
            self.demo_monitor.set_status(DemoState.error)
            return
        self.isExecuting = True
        self.demo_monitor.set_status(DemoState.running)

        d = msg.data.split(" ")
        t = 1.0
        if len(d) > 1:
            try:
                t = float(d[1])
            except ValueError:
                pass
        self.hand.setPose()
        for i in range(3):
            if i == 2: self.hand.setPose()
            self.moveWait(UP_JS,t=t)
            self.moveWait(LOW_JS,t=t)


        rospy.sleep(2.)
        self.isExecuting = False
        self.demo_monitor.set_status(DemoState.pause)


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


def test_SchnickSchnackSchnuckController():
    print("Hello world")
    # Test Hand Poses
    rospy.init_node("Test_SchnickSchnackSchnuckController")
    obj = SchnickSchnackSchnuckController()
    rospy.spin()


if __name__ == '__main__':
    # test_SchnickSchnackSchnuckHandModel()
    test_SchnickSchnackSchnuckController()
