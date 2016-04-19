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

import rospy
from std_msgs.msg import String
from DemoState import DemoState

""" @DemoStatus
General class to keep track of a demo status by publishing it via $(demo_name)_status
@author: Steve Grehl
"""

class DemoStatus(object):

    def __init__(self, str_name):
        """
        Default constructor
        :param str_name: name of the demo to monitor
        :type str_name: str
        """
        self.pub = rospy.Publisher("/"+str_name+'_status', String, queue_size=10)
        self.state = DemoState.unknown

    def set_status(self, state):
        """
        publishes the given status at the $(demo_name)_status topic
        :param state: state of the demo
        :type state: DemoState
        :return: -
        :rtype: None
        """
        self.state = state
        if not rospy.is_shutdown():
            str_status = DemoState.to_String(state)
            rospy.loginfo(str_status)
            self.pub.publish(str_status)

    def get_status(self):
        return self.state

if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        talker = DemoStatus("dummy")
        talker.set_status(DemoState.running)
        rospy.sleep(5.0)
        talker.set_status(DemoState.pause)
        rospy.sleep(5.0)
        talker.set_status(DemoState.stop)
        rospy.sleep(5.0)
        talker.set_status(DemoState.error)
    except rospy.ROSInterruptException:
        pass