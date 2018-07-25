#!/usr/bin/env python
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
import rospy
import math
from sensor_msgs.msg import JointState
import robotiq_s_model_control.msg as hand_msg

"""@package RobotiqJointStatePublisher
This module aims to pipe the joint state information provided by Robotiq into the tf by publishing it to the joint_state_publisher
@author: Steve Grehl
"""


class RobotiqJointStatePublisher(object):
    """
    An instance of this class pipes the joint_states reported by the Robotiq nodes to the tf using a external
    joint_state_publisher (see $(find tbf_gripper_base)/launch/hand_description.launch)
    """

    def __init__(self):
        """
        Default constructor: initilizes ROS communication (node, subscriber, and publisher)
            - defines the names in the tf and a default position of -1.0 for each of them
        @return:
        """
        rospy.init_node('RobotiqJointStatePublisher', anonymous=True)
        prefix = rospy.get_param('~prefix', default='default')
        input_topic = rospy.get_param('~sub_topic', default='SModelRobotInput')
        joint_topic = rospy.get_param('~pub_topic', default='joint_states_hand')
        # rospy.loginfo("[RobotiqJointStatePublisher.__init__()] prefix: %s" % prefix)

        rospy.Subscriber(name=input_topic, data_class=hand_msg.SModel_robot_input,
                         callback=self.onSModelInputMessage)
        self.publisher = rospy.Publisher(name=joint_topic, data_class=JointState, queue_size=10)
        self.names = ['finger_middle_joint_1', 'finger_1_joint_1', 'finger_2_joint_1','finger_middle_joint_2',
                      'finger_1_joint_2', 'finger_2_joint_2', 'finger_middle_joint_3', 'finger_1_joint_3',
                      'finger_2_joint_3', 'palm_finger_1_joint', 'palm_finger_2_joint']
        self.names = [prefix+name for name in self.names]
        self.position = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        # self.position = [-1.0, -1.0, -1.0, -1.0, -1.0]
        # self.velocity = [-1.0, -1.0, -1.0, -1.0, -1.0]
        # self.effort = [-1.0, -1.0, -1.0, -1.0, -1.0]

    def onSModelInputMessage(self, data):
        """
        triggered when a message was received from the Robotiq gripper via ROS (/SModelRobotInput)
        The function adjusts the given finger positions (0-255) for 3 fingers and the 'scissor' angle to the joint_angles
        in the model by calculating a angular. The offset values are estimated. The increment values are measured in a
        technical drawing.
        @param data: message of the type robotiq_s_model_control.msg.SModel_robot_input (see refering message definition at the robotiq package for further information)
        @return: None
        """
        # msg = hand_msg.SModel_robot_input(data)
        if (data.gACT == 0):
            return

        # phalanx 0 (from palm): 125- 55 (open-close)
        # phalanx 1: 0-(-90)
        # phalanx 2: 45-(-30)
        offset = math.radians(55.0) - math.radians(45)

        off_0 = math.radians(55.0) - math.radians(45)
        off_1 = math.radians(-90.0) + math.radians(90)
        off_2 = math.radians(-30.0)

        inc_0 = math.radians(70.0) / 255.0
        inc_1 = math.radians(90.0) / 255.0
        inc_2 = math.radians(75.0) / 255.0

        # phalanx 0
        self.position[0] = off_0 + data.gPOA * inc_0 # middle finger (A)
        self.position[1] = off_0 + data.gPOB * inc_0 # right finger (B) - connecters are the front 
        self.position[2] = off_0 + data.gPOC * inc_0 # left finger (C)

        # phalanx 1
        self.position[3] = off_1 + data.gPOA * inc_1
        self.position[4] = off_1 + data.gPOB * inc_1
        self.position[5] = off_1 + data.gPOC * inc_1

        # phalanx 2
        self.position[6] = off_2 + data.gPOA * inc_2
        self.position[7] = off_2 + data.gPOB * inc_2
        self.position[8] = off_2 + data.gPOC * inc_2

        # scissor angle
        scissor_angle = math.radians(23.0)
        sc_increment = scissor_angle / 255.0
        self.position[
            9] = scissor_angle / 2.0 - data.gPOS * sc_increment  # actual angle of the "scissor" [0-255] -> [-0.2793-0.2793] rad
        self.position[10] = -self.position[9]

        joint_msg = JointState()
        joint_msg.name = self.names
        joint_msg.position = self.position
        self.publisher.publish(joint_msg)
        # rospy.loginfo("[RobotiqJointStatePublisher.onSModelInputMessage()] published")


def main():
    """
    Start a RobotiqJointStatePublisher-instance and keep it running via rospy.spin()
    @return:
    """
    try:
        js_publisher = RobotiqJointStatePublisher()
        rospy.spin()
    except Exception, e:
        print("An errror occured")


if __name__ == '__main__': main()
