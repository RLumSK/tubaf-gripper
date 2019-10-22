#!/usr/bin/env python
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, TU Bergakademie Freiberg
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

"""
This script mimics a Robotiq S-model 3 finger gripper action server, using the GripperCommand
@author:grehl
"""

import rospy
import math
import actionlib
import control_msgs.msg
from sensor_msgs.msg import JointState


class DummyGripperAction(object):

    def __init__(self):
        self._action_name = rospy.get_param("gripper_action_server_name", "Robotiq3FGripperServer")
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        # Publish joint_states
        prefix = rospy.get_param('~prefix', default='default')
        joint_topic = rospy.get_param('~js_pub_topic', default='joint_states')
        self.publisher = rospy.Publisher(name=joint_topic, data_class=JointState, queue_size=10)
        self.names = ['finger_middle_joint_1', 'finger_1_joint_1', 'finger_2_joint_1','finger_middle_joint_2',
                      'finger_1_joint_2', 'finger_2_joint_2', 'finger_middle_joint_3', 'finger_1_joint_3',
                      'finger_2_joint_3', 'palm_finger_1_joint', 'palm_finger_2_joint']
        self.names = [prefix+name for name in self.names]
        self.position = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        self._current_position = 0.0
        self._as.start()

    def execute_cb(self, goal):
        """
        :param goal: target position for the gripper
        :type goal: control_msgs.msg.GripperCommandGoal
        :return:
        :rtype:
        """
        position = goal.command.position
        rg = True
        if position > 1.2:
            position = 1.
            rg = False
        if position < 0.0:
            position = 0.0
            rg = False

        steps = 10.0
        inc = (position - self._current_position) / steps
        while abs(self._current_position - position) > 0.000001:
            if rospy.is_shutdown():
                break
            rospy.logdebug("[DummyGripperAction.execute_cb()] current: %s, target: %s, inc: %s bool: %s"%
                           (self._current_position, position, inc, (self._current_position != position)))
            self._animate_joint_states(self._current_position)
            self._current_position += inc
            rospy.sleep(0.1)
        result = control_msgs.msg.GripperCommandResult()

        result.position = position
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = rg

        self._as.set_succeeded(result)

    def _animate_joint_states(self, target):
        """
        Publish joint_states of the gripper as its "moving" to the target position
        :param target: target position for the gripper
        :type target: float
        :return: -
        :rtype: -
        """
        # imported from RobotiqJointStatePublisher, mimic data fields of the gripper output:
        # 0.0 = open(0x00), 0.16 = close(0xFF)
        data = 255 - int(round(255 * target / 0.16))

        off_0 = math.radians(55.0) - math.radians(45)
        off_1 = math.radians(-90.0) + math.radians(90)
        off_2 = math.radians(-30.0)

        inc_0 = math.radians(70.0) / 255.0
        inc_1 = math.radians(90.0) / 255.0
        inc_2 = math.radians(75.0) / 255.0

        # phalanx 0
        self.position[0] = off_0 + data * inc_0  # middle finger (A)
        self.position[1] = self.position[0]  # right finger (B) - connectors are the front
        self.position[2] = self.position[0]  # left finger (C)

        # phalanx 1
        self.position[3] = off_1 + data * inc_1
        self.position[4] = self.position[3]
        self.position[5] = self.position[3]

        # phalanx 2
        self.position[6] = off_2 + data * inc_2
        self.position[7] = self.position[6]
        self.position[8] = self.position[6]

        # scissor angle
        scissor_angle = math.radians(23.0)
        sc_increment = scissor_angle / 255.0
        # 137 = basic
        self.position[9] = scissor_angle / 2.0 - 137 * sc_increment   # actual angle of the "scissor" [0-255] ->
                                                                            # [-0.2793-0.2793] rad
        self.position[10] = -self.position[9]

        joint_msg = JointState()
        joint_msg.name = self.names
        joint_msg.position = self.position
        self.publisher.publish(joint_msg)
        rospy.logdebug("[DummyGripperAction._animate_joint_states()] joint states published")


if __name__ == '__main__':
    # for a blueprint see:
    # http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
    rospy.init_node("gripper_action_server", log_level=rospy.INFO)
    DummyGripperAction()
    rospy.spin()
