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

from tbf_gripper_rqt.gripper_module import BasicGripperModel

import numpy as np

# [Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3]
HOME_POS = [0.0, -90, 0, -90, 0, 0]


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
        self.tf_listener = tf.TransformListener()

        self.joint_sub = rospy.Subscriber("/ur5/joint_states", JointState, self.onJs, queue_size=1)
        pose_topic = rospy.get_param("~pose_topic","/facePose")
        self.joint0_frame = "gripper_ur5_shoulder_link"
        self.target_pose_sub = rospy.Subscriber(pose_topic,PoseStamped,self.onPose,queue_size=1)

        self.program_pub = rospy.Publisher("/ur5/ur_driver/URScript", String, queue_size=1)

        # self.move_wait(HOME_POS, v=45, a=20)

        print "init done"
        rospy.sleep(0.5)

    def onJs(self, js):
        if time.time() - self.lasttime > 0.02:
            pp = list(js.position)
            pp[0] = js.position[2]
            pp[2] = js.position[0]

            self.cur_pos = np.rad2deg(pp)
            self.lasttime = time.time()

    def onPose(self,pose):
        ps = PointStamped()
        ps.header = pose.header
        ps.point = pose.pose.position
        print self.computeJoint0Angle(ps)
        pass


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

    def computeJoint0Angle(self,point_s):
        point_t = self.tf_listener.transformPoint(self.joint0_frame,point_s)
        phi = np.math.atan2(point_t.point.x,point_t.point.y)
        return phi



if __name__ == '__main__':
    rospy.init_node("Test_PnPwlanDemo")
    obj = PeopleTrackDemo()
    rospy.spin()