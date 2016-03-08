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
#author: grehl

import random
import signal
import sys

import rospy
from std_msgs.msg import String

from tbf_gripper_rqt.hand_module import RobotiqHandModel
from tbf_gripper_tools import Mover

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
UP_JS = [-4.65, -48.12, -95.24, -152.48, -93.91, -38.99]
LOW_JS = [-4.74, -15.95, -117.97, -51.06, -87.09, -46.08]

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class SchnickSchnackSchnuckHandModel(RobotiqHandModel):

    def __init__(self):
        super(SchnickSchnackSchnuckHandModel, self).__init__()

        # initilize Hand
        self.onActivationChanged(1)

        # set Speed and Force
        force = 255
        speed = 255
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

    def setStone(self):
        self.mdl_fingerA.onPositionRequestChanged(255)
        self.mdl_fingerB.onPositionRequestChanged(255)
        self.mdl_fingerC.onPositionRequestChanged(255)
        self.mdl_fingerS.onPositionRequestChanged(0)
        rospy.loginfo("SchnickSchnackSchnuckModel.setStone")

    def setScisscor(self):
        self.mdl_fingerA.onPositionRequestChanged(255)
        self.mdl_fingerB.onPositionRequestChanged(0)
        self.mdl_fingerC.onPositionRequestChanged(0)
        self.mdl_fingerS.onPositionRequestChanged(0)
        rospy.loginfo("SchnickSchnackSchnuckModel.setScissor")

    def setPaper(self):
        self.mdl_fingerA.onPositionRequestChanged(0)
        self.mdl_fingerB.onPositionRequestChanged(0)
        self.mdl_fingerC.onPositionRequestChanged(0)
        self.mdl_fingerS.onPositionRequestChanged(255)
        rospy.loginfo("SchnickSchnackSchnuckModel.setPaper")

    def setFountain(self):
        self.mdl_fingerA.onPositionRequestChanged(120)
        self.mdl_fingerB.onPositionRequestChanged(120)
        self.mdl_fingerC.onPositionRequestChanged(120)
        self.mdl_fingerS.onPositionRequestChanged(82)
        rospy.loginfo("SchnickSchnackSchnuckModel.setFountain")

    def setPose(self):
        decision = random.random()
        if decision > 0.4:
            if decision > 0.7:
                self.setStone()
            else:
                self.setScisscor()
        else:
            if decision > 0.1:
                self.setPaper()
            else:
                self.setFountain()
        self.sendROSMessage()
        rospy.sleep(3.)

class SchnickSchnackSchnuckController():

    def __init__(self):
        # ROS Anbindung
        self.sub = rospy.Subscriber("/schnick_cmd", String, self.execute, queue_size=1)
        self.isExecuting = True

        # Hand
        self.hand = SchnickSchnackSchnuckHandModel()
        rospy.sleep(2.)
        self.hand.setPose()

        # Arm
        self.ur5_mover = Mover.Mover('/pos_based_pos_traj_controller/follow_joint_trajectory')
        self.ur5_mover.move_home()
        self.move_down(10.0)

        self.isExecuting = False

    def execute(self, msg):
        rospy.loginfo(["SchnickDemo.py@SchnickSchnackSchnuckController.execute() - received msg: ", msg.data])
        return
        if msg != "start" or self.isExecuting:
            rospy.loginfo("Not executing Schnick,Schnack,Schnuck Demo")
            return
        self.hand.setPose()
        self.move_up(5.)
        self.move_down(5.)
        self.move_up(5.)
        self.move_down(5.)
        self.move_up(5.)
        self.move_down(5.)
        rospy.sleep(2.)

    def move_down(self, duration=0.5):
        self.ur5_mover.move_and_wait(LOW_JS, duration)

    def move_up(self, duration=0.5):
        self.ur5_mover.move_and_wait(UP_JS, duration)

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
    #test_SchnickSchnackSchnuckHandModel()
    test_SchnickSchnackSchnuckController()
