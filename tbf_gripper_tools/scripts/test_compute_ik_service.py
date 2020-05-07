#!/usr/bin/python
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, TU Bergakademie Freiberg
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
# Author: grehl
import rospy

from autonomy.MoveitInterface import MoveitInterface
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse


def service_call(request):
    srv_name = 'compute_fk'
    rospy.wait_for_service(srv_name)
    rospy.loginfo("Service call: %s", srv_name)
    try:
        srv_call = rospy.ServiceProxy(srv_name, GetPositionFK)
        response = srv_call.call(request)  # type: GetPositionFKResponse
        if response.error_code.val != 1:
            rospy.logwarn("Error code: %s", response.error_code)
        return response
    except rospy.ServiceException:
        rospy.logwarn("Service call failed")
        return None


if __name__ == '__main__':
    rospy.init_node("EquipmentTask", log_level=rospy.DEBUG)
    mvit = MoveitInterface()
    rospy.loginfo("%s", mvit)
    fk_request = GetPositionFKRequest()
    fk_request.header.stamp = rospy.Time.now()
    fk_request.fk_link_names = mvit.robot.get_link_names(mvit.group.get_name())
    fk_request.robot_state = mvit.robot.get_current_state()
    fk_response = service_call(fk_request)
    rospy.loginfo("%s", fk_response)

    # The end-effector pose of our forward kinematic is the target pose for our inverse kinematic
    result = mvit.get_ik(fk_response.pose_stamped[-1])
    rospy.loginfo("%s", result)
    rospy.spin()
