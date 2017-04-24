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
from tbf_gripper_tools.DemoStatus import *
import numpy as np
import thread

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
HOME_POS = [0.0, -90, 0, -90, 0, 0]

#WLAN_POS_2
WAYPOINTS = {
    'home':             HOME_POS,
    'top_up':           [-176,  -78,  15,  61,  87,  45],
    'top_pickup':       [-175.77, -94,  62,  35,  90,  45],
    # 'top_up2':          [-176,  -78,  15,  61,  87,  45],
    'front_up':         [-2.5,  -79,  38,  37,  85,  45],
    # 'front_place':    [-2.3,  -7.8,   95.65,  -91.32, 84.42, 45],
    'front_pickup':     [-2.3,   -6,  94, -92,  84,  45],
}

# WAYPOINTS = [
#     [-176.36, -69.79,   2.09,   67.31, 87.44, 45],
#     [-176.36, -96.82,  69.03,   27.41, 87.77, 45],
#     [-174.67, -90.60,  50.88,   35.38, 87.62, 45],
#     [  -2.48, -79.31,  38.12,   36.86, 85.16, 45],
#     [  -2.3,  -6.15,   94.22,  -91.52, 84.42, 45]
#     # [  -2.28,  -6.98,   94.95,  -91.42, 84.44, 45]
# ]

class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"

# HOME_PROGRAM = \
# """movej(%(homepos)s, a=%(a)f, v=%(v)f)
# movej(%(lowpos)s, a=%(a)f, v=%(v)f)
# """ % {'homepos': pos2str(HOME_POS), 'lowpos': pos2str(LOW_JS),
#        'a': np.deg2rad(20), 'v': np.deg2rad(45)}


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class PickAndPlaceWlanDemo:
    """
    Class to perform a simple Pick and Place demo
    """
    def __init__(self):
        """
        Default constructor, start ROS, hand_model and demo_monitoring
        """

        # ROS Anbindung
        self.sub = rospy.Subscriber("/pnp_demo_cmd", String, self.execute, queue_size=1)

        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.onJs, queue_size=1)

        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)
        self.move_dur = 1.0
        self.cur_pos = np.zeros((6,))
        self.lasttime = time.time()
        self.last_start = time.time()

        self.demo_monitor = DemoStatus("pnp_demo")
        self.exec_thread = None
        rospy.sleep(0.5)

        if rospy.get_param("~no_init", False):
            rospy.logwarn("init skipped!")
            self.demo_monitor.set_status(DemoState.error)
        else:
            self.run_as_process(PickAndPlaceWlanDemo.initialise)

    def run_as_process(self,function):
        """
        @param function: the function to start
        @param args: tuple of function args
        """
        # p = Process(target=function, args=(self,))
        # p.start()
        # self.exec_thread = p
        self.exec_thread = thread.start_new_thread(function,(self,))

    def onJs(self, js):
        if time.time() - self.lasttime > 0.02:
            pp = list(js.position)
            pp[0] = js.position[2]
            pp[2] = js.position[0]

            self.cur_pos = np.rad2deg(pp)
            self.lasttime = time.time()


    def move_wait(self, pose, goal_tolerance=0.5, v=None, a=None, t=None, move_cmd="movej"):
        rospy.loginfo("move and wait... %s",str(pose))
        prog = move_cmd+"(%s" % pos2str(pose)
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
            dst = np.abs(np.subtract(pose,self.cur_pos))
            if np.max(dst) < goal_tolerance:
                break
            rospy.sleep(0.02)

    def perform_demo(self):
        spd = 20
        self.demo_monitor.set_status(DemoState.running)
        # Move to Station on top of the Robot starting at HOME position
        self.move_wait(WAYPOINTS['home'], v=45, a=20)
        self.hand.openGripper()
        self.move_wait(WAYPOINTS['top_up'], v=spd)
        self.move_wait(WAYPOINTS['top_pickup'], t=2.4, move_cmd="movel")

        # Grasp station
        self.hand.closeGripper()
        rospy.sleep(2.)
        self.move_wait(WAYPOINTS['top_up'], t=1.6, move_cmd="movel")

        # Set station
        self.move_wait(WAYPOINTS['front_up'], v=spd)
        # self.move_wait(WAYPOINTS['front_place'], move_cmd="movel")
        self.move_wait(WAYPOINTS['front_pickup'], move_cmd="movel")
        self.hand.openGripper()
        rospy.sleep(2.)

        # Move to HOME position
        self.move_wait(WAYPOINTS['front_up'], move_cmd="movel")
        self.move_wait(WAYPOINTS['home'], v=spd, a=20)

        rospy.loginfo("arrived at home with station, waiting 5 secs")
        rospy.sleep(5.0)

        # Grasp station again
        self.move_wait(WAYPOINTS['front_up'], v=spd)
        self.move_wait(WAYPOINTS['front_pickup'], move_cmd="movel")
        self.hand.closeGripper()
        rospy.sleep(2.)

        # Set station on top of the robot
        self.move_wait(WAYPOINTS['front_up'], move_cmd="movel")
        self.move_wait(WAYPOINTS['top_up'], v=spd)
        self.move_wait(WAYPOINTS['top_pickup'], move_cmd="movel")
        self.hand.openGripper()
        rospy.sleep(2.)

        # Back to HOME position
        self.move_wait(WAYPOINTS['top_up'], t=2.4, move_cmd="movel")
        self.move_wait(WAYPOINTS['home'], v=spd, a=20)

        self.demo_monitor.set_status(DemoState.stop)
        self.exec_thread = None

    def initialise(self):
        rospy.loginfo("initialisation... moving to home pose and opening gripper")
        rospy.sleep(1.0)
        # Arm
        rospy.loginfo("move home...")
        self.move_wait(HOME_POS, v=45, a=20)

        rospy.loginfo("arrived at home, opening gripper in 3 secs...")
        # Hand
        rospy.sleep(3.0)
        self.hand = BasicGripperModel()
        self.hand.mdl_fingerA.rSP = 255
        self.hand.mdl_fingerA.rFR = 255
        self.hand.openGripper()
        rospy.loginfo("wait 20s for the hand to initialise")
        rospy.sleep(20.0)
        rospy.loginfo("init done")
        self.demo_monitor.set_status(DemoState.initialized)
        self.exec_thread = None

    def execute(self, msg):
        """
        pick and place and pick and place the WLAN station
        :param msg: string message with any content, just to trigger the execution
        :type msg: String
        :return: -
        :rtype: None
        """
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
                    self.run_as_process(PickAndPlaceWlanDemo.initialise)
                    return
                self.last_start = time.time()
                self.demo_monitor.set_status(DemoState.unknown)
                rospy.loginfo("error state, start received...   Press start again within 2 secs to perform reinitialisation")
            return

        if msg.data.startswith("start"):
            if self.exec_thread is None:
                self.run_as_process(PickAndPlaceWlanDemo.perform_demo)
                rospy.loginfo("starting demo...")
            else:
                rospy.logwarn("demo is already running, ignoring start command!")


        rospy.loginfo("Not executing Pick and Place Demo - received:" + msg.data)



if __name__ == '__main__':
    print("Hello world")
    rospy.init_node("PnPwlanDemo")
    obj = PickAndPlaceWlanDemo()
    if (rospy.get_param("~autostart",True)):
        rospy.loginfo("autostart is true, waiting for controller to initialise, then starting demo")
        while not obj.demo_monitor.get_status() == DemoState.initialized:
            time.sleep(1.0)
        rospy.loginfo("autostarting demo")
        obj.execute(String("start"))

    rospy.spin()