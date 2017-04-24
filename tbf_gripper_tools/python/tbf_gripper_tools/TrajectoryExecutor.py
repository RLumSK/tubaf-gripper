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
# author: donner

import rospy
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

DEFAULT_JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
DEFAULT_ACTION_TARGET = "pos_based_pos_traj_controller/follow_joint_trajectory"
DEFAULT_STATE_TOPIC = "joint_states"

HOME_POINT = [0.0, -90, 0, -90, 0, 0]

class TrajectoryExecutor:
    def __init__(self, action_target=DEFAULT_ACTION_TARGET, state_topic=DEFAULT_STATE_TOPIC,
                 joint_names=DEFAULT_JOINT_NAMES):
        self.joint_names = joint_names
        self.lastPose = None
        self.jss = rospy.Subscriber(state_topic, JointState, self.on_js, queue_size=1)

        self.ac = actionlib.SimpleActionClient(action_target, FollowJointTrajectoryAction)
        rospy.loginfo("waiting for action server")
        self.ac.wait_for_server()
        rospy.loginfo("Action interface ready")

        self.cur_idx = 0

    def on_js(self, js):
        pp = list(js.position)
        pp[0] = js.position[2]
        pp[2] = js.position[0]

        self.lastPose = np.rad2deg(pp)
        pass

    def _actionCb(self, data=None, stuff=None):
        # print(data, stuff)
        pass

    def move_home(self):
        self.move_and_wait(HOME_POINT, 5.)

    def move_and_wait(self, *poses):
        tra = JointTrajectory()
        tra.points = list()
        tra.joint_names = self.joint_names
        for pose, dur in poses:
            pt = JointTrajectoryPoint(positions=np.deg2rad(pose), time_from_start=rospy.Duration.from_sec(dur))
            tra.points.append(pt)
        # print "------", type(tra.points)

        go = FollowJointTrajectoryGoal()
        go.trajectory = tra
        self.ac.send_goal(go, self._actionCb, self._actionCb, self._actionCb)
        self.ac.wait_for_result()

    def compute_dur(self, target_pose_deg, speeds_deg):
        cur_pose = self.lastPose.copy()
        distance = np.abs(np.subtract(target_pose_deg, cur_pose))
        timings = np.divide(distance, speeds_deg)

        return np.max(timings)


def main():
    rospy.init_node("Test_Mover")

    prefix = "gripper_ur5_"
    names = map(lambda s: prefix + s, DEFAULT_JOINT_NAMES)

    speeds = [90, 45, 45, 45, 45, 45]

    obj = TrajectoryExecutor(joint_names=names)
    target = [-133.8, -75.0, -100, -90.0, 17.71, 45]
    dur = obj.compute_dur(target, speeds) + 0.5
    obj.move_and_wait(target_pose_deg=target, dur=dur)


if __name__ == '__main__':
    main()
