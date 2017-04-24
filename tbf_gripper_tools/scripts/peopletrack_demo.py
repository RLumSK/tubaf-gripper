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
from geometry_msgs.msg import PoseStamped, PointStamped
import tf
import thread

from tbf_gripper_rqt.gripper_module import BasicGripperModel

import numpy as np

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
HOME_POSE = [0.0, -90, 0, -90, 0, 0]
MONITOR_POSE = [0, -90, 10, -95, -90, -45]

PAN_LIMITS = (-35, 35)

MODE_IDLE = 0
MODE_ALIGN = 1
MODE_GIVE = 2
MODE_RESET = 3


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class PeopleTrackDemo:
    def __init__(self):
        # ROS Anbindung
        self.lasttime = time.time()
        self.direction = 1  # 1 or -1
        self.pan_target = 0
        self.tf_listener = tf.TransformListener()

        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.onJs, queue_size=1)
        pose_topic = rospy.get_param("~pose_topic", "/facePose")
        self.joint0_frame = "gripper_ur5_shoulder_link"
        self.target_pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.onPose, queue_size=1)

        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)

        self.hand = BasicGripperModel()
        self.hand.moveGripperTo(135)

        rospy.sleep(0.5)
        self.mode = MODE_IDLE
        self.logic_thread = thread.start_new_thread(PeopleTrackDemo.perform_logic_thread, (self,))

    def onJs(self, js):
        if time.time() - self.lasttime > 0.02:
            pp = list(js.position)
            pp[0] = js.position[2]
            pp[2] = js.position[0]

            self.cur_pos = np.rad2deg(pp)
            self.lasttime = time.time()

    def onPose(self, pose):
        ps = PointStamped()
        ps.header = pose.header
        ps.point = pose.pose.position

        if ps.point.z == 0:
            self.pan_target = 0
            self.mode = MODE_IDLE
            return

        if ps.point.z > 1.2:
            self.pan_target = self.computeJoint0Angle(ps)
            self.mode = MODE_ALIGN
            return

        if ps.point.z <= 1.2:
            self.pan_target = self.computeJoint0Angle(ps)
            self.mode = MODE_GIVE

        pass

    def move_wait(self, pose, goal_tolerance=0.5, v=None, a=None, t=None, move_cmd="movej", ignore_wait=False):
        prog = move_cmd + "(%s" % pos2str(pose)
        if a is not None:
            prog += ", a=%f" % np.deg2rad(a)
        if v is not None:
            prog += ", v=%f" % np.deg2rad(v)
        if t is not None:
            prog += ", t=%f" % t
        prog += ")"
        self.program_pub.publish(prog)
        while not ignore_wait:
            dst = np.abs(np.subtract(pose, self.cur_pos))
            if np.max(dst) < goal_tolerance:
                break
            rospy.sleep(0.02)

    def computeJoint0Angle(self, point_s):
        point_t = self.tf_listener.transformPoint(self.joint0_frame, point_s)
        phi = np.math.atan2(point_t.point.x, point_t.point.y)
        return phi

    def perform_logic_thread(self):
        self.move_wait(HOME_POSE)
        rospy.loginfo("reached home pose, going to monitoring pose")
        self.move_wait(MONITOR_POSE)
        rospy.loginfo("reached monitoring pose, waiting for poses, performing idle tracking")
        timer = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.mode == MODE_IDLE:
                pan = self.cur_pos[0]
                pan += 5*self.direction
                # print pan
                if pan < PAN_LIMITS[0] or pan > PAN_LIMITS[1]:
                    self.direction *= -1
                    pan += 10*self.direction  # double step width, rverse previous step and perform new one
                newpose = list(MONITOR_POSE)  # creates mutable copy
                newpose[0] = pan

                self.move_wait(newpose,t=1.1,ignore_wait=True)

            elif self.mode == MODE_ALIGN:
                newpose = list(MONITOR_POSE)  # creates mutable copy
                newpose[0] = self.pan_target

                self.move_wait(newpose, t=1.1, ignore_wait=True)
                pass

            elif self.mode == MODE_GIVE:
                newpose = list(MONITOR_POSE)  # creates mutable copy
                newpose[0] = self.pan_target
                newpose[1] -=5
                newpose[2] +=25
                newpose[3] -=20
                self.move_wait(newpose, t=4.5)

            timer.sleep()


if __name__ == '__main__':
    rospy.init_node("Test_PnPwlanDemo")
    obj = PeopleTrackDemo()
    rospy.spin()
