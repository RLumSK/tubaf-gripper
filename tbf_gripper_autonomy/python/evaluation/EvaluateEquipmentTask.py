#!/usr/bin/python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, TU Bergakademie Freiberg
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

import rospy
import os

from geometry_msgs.msg import PoseStamped as PoseStamped
from geometry_msgs.msg import Pose as Pose
from sensor_msgs.msg import Image as Image
from std_msgs.msg import Float32 as Float
from std_msgs.msg import String as String
from std_msgs.msg import Int32 as Int32


class EquipmentTask(object):
    """
    Evaluate the handling of Smart Equipment, mainly implemented in tbf_gripper_autonomy.autonomy.EquipmentTask.py
    Members of this class may be accessed public
    """

    def __init__(self):
        """
        Default constructor
        """
        self.dct_trajectory = {}
        self.dct_planing_time = {}
        self.dct_planner = {}
        self.dct_attempts = {}
        self.dct_rel_time = {}
        self.estimated_set_pose = PoseStamped()
        self.sensed_set_pose = PoseStamped()
        self.intermediate_set_pose = PoseStamped()
        self.sensed_pose_confidence = 0.0
        self.grasp_relation = Pose()

        self.dct_rgb_img = {}
        self.dct_dpt_img = {}

        self._paused_since = False
        self.t_start = rospy.Time()
        self._t_pause = rospy.Duration(0, 0)

        self.t_in_s = 0.0

        self._rgb_topic = rospy.get_param("~rgb_topic", "/gripper_d435/color/image_rect_color")
        self._dpt_topic = rospy.get_param("~depth_topic", "/gripper_d435/depth/image_rect_raw")

    def add_moveit_plan_information(self, key, plan, duration, attempts, time):
        """
        Store information from MoveIt for a given key
        :param key: key for the dictionaries
        :param plan: RobotTrajectory
        :param duration: given planing time
        :param attempts: used number of attempts
        :param time: Time
        :return:
        """
        if key in self.dct_trajectory:
            self.dct_trajectory[key].append(plan)
            self.dct_planing_time[key].append(duration)
            self.dct_attempts[key].append(attempts)
            self.dct_rel_time[key].append(time)
        else:
            self.dct_trajectory[key] = [plan]
            self.dct_planing_time[key] = [duration]
            self.dct_attempts[key] = [attempts]
            self.dct_rel_time[key] = [time]

    def pause(self):
        """
        Pause timing
        :return:
        """
        if self._paused_since:
            rospy.logwarn("[EquipmentTask.pause()] already in pause")
            return
        self._paused_since = rospy.Time.now()

    def resume(self):
        """
        Resume timing
        :return:
        """
        end_time = rospy.Time.now()
        if not self._paused_since:
            rospy.logwarn("[EquipmentTask.resume()] not in pause")
            return
        self._t_pause = self._t_pause + (end_time - self._paused_since)  # Time minus Time is a Duration
        self._paused_since = False

    def calc_time(self, now=None):
        """
        Calculate timing with the saved pause
        :return:
        """
        if now is None:
            now = rospy.Time.now()
        duration = now - self.t_start - self._t_pause
        t_in_s = duration.to_sec()
        return t_in_s

    def store_img(self, key):
        """
        Save one image from the rgb and depth topic in the dictionaries under the given key
        :param key:
        :return:
        """
        self.dct_rgb_img[key] = rospy.wait_for_message(self._rgb_topic, Image)
        self.dct_dpt_img[key] = rospy.wait_for_message(self._dpt_topic, Image)

    def save_as_bag(self, file_path):
        """
        Save all information in a bag file under the given path
        :param file_path: file path of the bag file
        :return: -
        """
        import rosbag

        file_path = check_path(file_path)
        bag = rosbag.Bag(file_path, 'w')
        wait_duration = 0.05
        try:
            for key in self.dct_trajectory:
                try:
                    for item in self.dct_trajectory[key]:
                        bag.write('trajectory/' + key, item)
                        rospy.sleep(wait_duration)
                    for item in self.dct_planing_time[key]:
                        bag.write('planing_time/' + key, Float(item))
                        rospy.sleep(wait_duration)
                    for item in self.dct_rel_time[key]:
                        bag.write('timing/' + key, Float(item))
                        rospy.sleep(wait_duration)
                    for item in self.dct_planner[key]:
                        bag.write('planner/' + key, String(item))
                        rospy.sleep(wait_duration)
                    for item in self.dct_attempts[key]:
                        bag.write('attempts/' + key, Int32(item))
                        rospy.sleep(wait_duration)
                except KeyError as ke:
                    rospy.logerr("[EvaluateEquipmentTask.write_bag()] KeyError: %s \n Allowed keys per dict:"
                                 "\n\t %s: %s\n\t %s: %s\n\t %s: %s\n\t %s: %s\n\t %s: %s" %
                                 (ke.message,
                                  "dct_trajectory", self.dct_trajectory.keys(),
                                  "dct_planing_time", self.dct_planing_time.keys(),
                                  "dct_rel_time", self.dct_rel_time.keys(),
                                  "dct_planner", self.dct_planner.keys(),
                                  "dct_attempts", self.dct_attempts.keys()
                                  ))
            for key in self.dct_rgb_img:
                bag.write('rgb/' + key, self.dct_rgb_img[key])
                bag.write('depth/' + key, self.dct_dpt_img[key])
            import string
            bag.write('traj_keys', String(";".join(self.dct_trajectory.keys())))
            bag.write('img_keys', String(";".join(self.dct_rgb_img.keys())))
            bag.write('pose/estimated', self.estimated_set_pose)
            bag.write('pose/intermediate', self.intermediate_set_pose)
            bag.write('pose/sensed', self.sensed_set_pose)
            bag.write('grasp_relation', self.grasp_relation)
            bag.write('sensed_pose_confidence', Float(self.sensed_pose_confidence))
            bag.write('t_in_s', Float(self.t_in_s))
        finally:
            bag.close()


def check_path(path):
    """
    Make sure the path exists
    :param path: file path
    :return: cleaned path
    """
    file_path = os.path.abspath(os.path.expanduser(path))
    d = str(os.path.join(os.path.split(file_path)[:-1])[0])
    if not os.path.exists(d):
        os.makedirs(d)
    return file_path


if __name__ == '__main__':
    rospy.init_node("evaluate_autonomy", log_level=rospy.DEBUG)
    # Init
    obj = EquipmentTask()
    obj.t_start = rospy.Time.now()
    rospy.sleep(1.0)
    obj.store_img("test")
    obj.pause()
    rospy.sleep(1.0)
    obj.resume()
    rospy.sleep(1.0)
    obj.t_in_s = obj.calc_time()
    obj.save_as_bag("~/test/unknown_dir/test.bag")
