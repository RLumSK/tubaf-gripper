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

import grasping.hand
import numpy as np
import thread


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class WaterSampleTask:
    """
    Class to get a water sample
    """
    def __init__(self):
        """
        Default constructor, start ROS, hand_model and demo_monitoring
        """

        # ROS Anbindung
        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.on_joint_states, queue_size=1)
        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)
        self.move_dur = 1.0
        self.cur_pos = np.zeros((6,))
        self.last_time = time.time()
        self.last_start = time.time()
        self.exec_thread = None
        self.hand_controller = grasping.hand.HandController()
        rospy.loginfo("water_sample_task.py: WaterSampleTask(): initilized hand")

        self.waypoints = rospy.get_param("~waypoints")
        self.backup_pose = rospy.get_param("~backup_pos")
        self.arm_speed = rospy.get_param("~arm_speed", 20.0)
        self.arm_acceleration = rospy.get_param("~arm_acceleration", 20.0)

        rospy.sleep(0.5)
        self.run_as_process(WaterSampleTask.initialise)

    def run_as_process(self, function):
        """
        Run a given function as Thread process
        :param function: function to execute
        :type function: function
        :return: -
        :rtype: -
        """
        if self.exec_thread is not None:
            rospy.logwarn("WaterSampleTask.run_as_process() self.exec_thread=%s", self.exec_thread)
            return
        self.exec_thread = thread.start_new_thread(function, (self,))

    def on_joint_states(self, js):
        """
        Callback for the joint_states topic of the UR5
        :param js: joint states
        :type js: double
        :return: -
        :rtype: -
        """
        # rospy.loginfo("WaterSampleTask.on_joint_states(): received jopint states: %s", js)
        if time.time() - self.last_time > 0.02:
            pp = list(js.position)
            # pp[0] = js.position[2]
            # pp[2] = js.position[0]

            self.cur_pos = np.rad2deg(pp)
            self.last_time = time.time()

    def move_wait(self, pose, goal_tolerance=0.5, v=None, a=None, t=None, move_cmd="movej"):
        """
        Move a UR5 to a given pose using the URScript interface
        :param pose: target pose as joint values
        :type pose: (6x double)
        :param goal_tolerance: tolerance for the target
        :type goal_tolerance: double
        :param v: velocity for the joint movement in deg/s
        :type v: double
        :param a: acceleration for the joint values in deg/s2
        :type a: double
        :param t: time
        :type t: double
        :param move_cmd: move command from URScript
        :type move_cmd: str [movej, movel, movep]
        :return: -
        :rtype: -
        """
        rospy.loginfo("WaterSampleTask.move_wait(): %s to:\n %s", move_cmd,  str(pose))
        program = move_cmd + "(%s" % pos2str(pose)
        if a is not None:
            program += ", a=%f" % np.deg2rad(a)
        if v is not None:
            program += ", v=%f" % np.deg2rad(v)
        if t is not None:
            program += ", t=%f" % t
        program += ")"
        if self.exec_thread is not None:
            self.program_pub.publish(program)
        else:
            raise InterruptError("Thread interrupted")
        while True:
            dst = np.abs(np.subtract(pose, self.cur_pos))
            max_dst = np.max(dst)
            if max_dst < goal_tolerance:
                break
            rospy.logdebug("WaterSampleTask.move_wait(): self.cur_pos=%s", self.cur_pos)
            rospy.logdebug("WaterSampleTask.move_wait():         pose=%s", pose)
            rospy.logdebug("WaterSampleTask.move_wait(): max_dst = %s (tol=%s)", max_dst, goal_tolerance)
            rospy.sleep(0.02)

    def perform(self):
        """
        Hard coded task to pickup a water sample station from the robot, take a sample and return it to the robot
        :return: -
        :rtype: -
        """
        rospy.loginfo("WaterSampleTask.perform(): Move to Station on top of the Robot starting at HOME position")
        # Move to Station on top of the Robot starting at HOME position
        self.move_wait(self.waypoints["home_pose"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")
        self.hand_controller.openHand()
        self.move_wait(self.waypoints["by_resting"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")
        self.move_wait(self.waypoints["pick_up"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movel")

        rospy.loginfo("WaterSampleTask.perform(): Grasp station")
        # Grasp station
        self.hand_controller.closeHand()
        rospy.sleep(2.)
        self.move_wait(self.waypoints["move_free"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")
        self.move_wait(self.waypoints["hover_pose"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")

        rospy.loginfo("WaterSampleTask.perform(): Set station")
        # Set station
        self.move_wait(self.waypoints["bottom_pose"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movel")

        rospy.loginfo("WaterSampleTask.perform(): Take sample -automatic-")
        # Take sample -automatic-
        rospy.sleep(30.)

        rospy.loginfo("WaterSampleTask.perform(): Lift station")
        # Lift station
        self.move_wait(self.waypoints["hover_pose"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movel")

        rospy.loginfo("WaterSampleTask.perform(): Move to origin")
        # Move to origin
        self.move_wait(self.waypoints["move_free"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")
        self.move_wait(self.waypoints["pick_up"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")

        rospy.loginfo("WaterSampleTask.perform(): Release station")
        # Release station
        self.hand_controller.openHand()
        rospy.sleep(2.)

        rospy.loginfo("WaterSampleTask.perform(): Move back to home station")
        # Move back to home station
        self.move_wait(self.waypoints["by_resting"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movel")
        self.move_wait(self.waypoints["home_pose"], v=self.arm_speed, a=self.arm_acceleration, move_cmd="movej")
        self.hand_controller.closeHand()
        rospy.sleep(2.)
        self.exec_thread = None

    def initialise(self):
        """
        Initilise by opening the hand and moving to a given backup_pose
        :return: -
        :rtype: -
        """
        rospy.loginfo("WaterSampleTask.initialisation(): Moving to home pose and opening gripper")
        # Arm
        rospy.loginfo("WaterSampleTask.initialisation(): Move to backup_pose...")
        self.move_wait(self.backup_pose, v=45, a=20)
        rospy.loginfo("WaterSampleTask.initialisation(): Arrived at backup_pose, opening gripper...")

        # Hand
        rospy.sleep(3.0)
        self.hand_controller.openHand()
        rospy.loginfo("WaterSampleTask.initialisation(): Wait 20s for the hand to initialise")
        rospy.sleep(20.0)
        rospy.loginfo("WaterSampleTask.initialisation(): Init done")
        self.exec_thread = None

    def start(self):
        """
        Start the water Sample Task
        :return: -
        :rtype: -
        """
        self.run_as_process(WaterSampleTask.perform)

if __name__ == '__main__':
    rospy.init_node("WaterSampleTask")
    obj = WaterSampleTask()
    while obj.exec_thread is not None:
        rospy.sleep(0.5)
    obj.start()
    rospy.spin()
