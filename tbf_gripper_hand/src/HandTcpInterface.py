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

import socket
import rospy
from tbf_gripper_rqt.gripper_module import BasicGripperModel

class HandTcpInterface:
    def __init__(self, server_ip='127.0.0.1', port=59995):
        #setup gripper (starts a ROS node)
        self.gripper = BasicGripperModel()
        # get parameter from ROS
        self.prefix = rospy.get_param("server_ip", server_ip)
        self.prefix = rospy.get_param("port", port)

        # setup tcp
        # https://wiki.python.org/moin/UdpCommunication
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((server_ip, port))


        self.run()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            data = self.s.recv(3)
            if not data:
                continue
            rospy.logdebug("GripperTcpController: received data - %s" % data)
            self.control(data)
            rate.sleep()

    def control(self, msg):
        if msg == "_o_":
            rospy.loginfo("GripperTcpController: open gripper")
            self.gripper.openGripper()
        elif msg == "_c_":
            rospy.loginfo("GripperTcpController: close gripper")
            self.gripper.closeGripper()

        else:
            as_number = int(msg)
            if as_number > -1 and as_number < 256:
                rospy.loginfo("GripperTcpController: set gripper to %s" % as_number)
                self.gripper.moveGripperTo(as_number)
            else:
                rospy.logwarn("GripperUdpController - unknown message receives: " + str(msg))


def main():
    print("Hello world")
    obj = HandTcpInterface(server_ip='192.168.2.35')


if __name__ == '__main__':
    main()
