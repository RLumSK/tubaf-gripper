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
# author: grehl

import signal
import sys

import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from tbf_gripper_rqt.gripper_module import BasicGripperModel

import numpy as np

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
HOME_POS = [0.0, -90, 0, -90, 0, 0]
WAYPOINTS = [
    [-176.36, -69.79,   2.09,   67.31, 87.44, 45.24],
    [-176.36, -96.82,  69.03,   27.41, 87.77, 45.04],
    [-174.67, -90.60,  50.88,   35.38, 87.62, 47.20],
    [  -2.48, -79.31,  38.12,   36.86, 85.16, 47.20],
    [  -3.63,  -3.75, 113.73, -114.39, 83.54, 48.24]
]

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

class PickAndPlaceWlanDemo:

    def __init__(self):
        # ROS Anbindung
        self.sub = rospy.Subscriber("/pnpwlan_cmd", String, self.execute, queue_size=1)

        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.onJs, queue_size=1)

        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)
        self.isExecuting = True
        self.lasttime = time.time()

        # Hand
        self.hand = BasicGripperModel()
        self.hand.mdl_fingerA.rSP = 255
        self.hand.mdl_fingerA.rFR = 255
        self.hand.openGripper()


        self.move_dur = 1.0
        self.cur_pos = np.zeros((6,))

        # Arm
        self.isExecuting = False

        rospy.sleep(0.5)
        prg = HOME_PROGRAM.replace("\n","\t")
        # for line in HOME_PROGRAM.strip().split('\n'):
        #     self.program_pub.publish(line)
        #     rospy.sleep(3.5)

        self.move_wait(HOME_POS, v=45, a=20)

        print "init done"
        rospy.sleep(0.5)

    def move_wait(self, pose, goal_tolerance=0.5, v=None, a=None, t=None, move_cmd="movej"):
        prog = move_cmd+"(%s" % pos2str(pose)
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
            rospy.loginfo("Not executing Pick and Place Demo - action pending")
            return
        if not msg.data.startswith("start"):
            rospy.loginfo("Not executing Pick and Place Demo - received:" + msg.data)
            return
        self.isExecuting = True
        # Move to Station on top of the Robot starting at HOME position
        self.move_wait(HOME_POS, v=45, a=20)
        self.hand.openGripper()
        self.move_wait(WAYPOINTS[0])
        self.move_wait(WAYPOINTS[1], move_cmd="movel")

        # Grasp station
        self.hand.closeGripper()
        rospy.sleep(2.)
        self.move_wait(WAYPOINTS[2], move_cmd="movel")

        # Set station
        self.move_wait(WAYPOINTS[3])
        self.move_wait(WAYPOINTS[4], move_cmd="movel")
        self.hand.openGripper()
        rospy.sleep(2.)

        # Move to HOME position
        self.move_wait(WAYPOINTS[3], move_cmd="movel")
        self.move_wait(HOME_POS, v=45, a=20)

        # Grasp station again
        self.move_wait(WAYPOINTS[3])
        self.move_wait(WAYPOINTS[4], move_cmd="movel")
        self.hand.closeGripper()
        rospy.sleep(2.)

        # Set station on top of the robot
        self.move_wait(WAYPOINTS[3], move_cmd="movel")
        self.move_wait(WAYPOINTS[2])
        self.move_wait(WAYPOINTS[1], move_cmd="movel")
        self.hand.openGripper()
        rospy.sleep(2.)

        # Back to HOME position
        self.move_wait(WAYPOINTS[0], move_cmd="movel")
        self.move_wait(HOME_POS, v=45, a=20)

        self.isExecuting = False


if __name__ == '__main__':
    obj = PickAndPlaceWlanDemo()
    obj.execute()