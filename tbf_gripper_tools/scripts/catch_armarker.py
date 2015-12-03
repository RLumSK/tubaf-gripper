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

# http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
import sys
import math
import random

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as transform
import ar_track_alvar_msgs.msg as ar_msg

from tbf_gripper_hand import GraspingMode
import GripperInterface
import GripperState

MARKER_ID = 42
AR_TOPIC = "/ar_pose_marker"
FRAME_ID = "camera_link"
X_DISTANCE = 0.1

def main():
    """
    Start a ARCatcher-instance and keep it running via rospy.spin()
    @return:
    """
    try:
        random.seed()
        gripper = GripperInterface()
        gripper.activate()
        rospy.sleep(10.0)
        # handle the ROS connection
        rospy.Subscriber(name=AR_TOPIC, data_class=ar_msg.AlvarMarkers, callback="onNewAlvarMarkers",
                         callback_args=gripper)
        rospy.spin()
    except Exception, e:
        print("An errror occured: " + str(e))


def onNewAlvarMarkers(msg, gripper):
    if gripper.state is not GripperState.GripperState.active:
            rospy.loginfo("catch_armarker.py@onNewAlvarMarkers: gripper not ready, state is: %s" % gripper.state)
            return
    if len(msg.markers) == 0:
            rospy.logwarn("catch_armarker.py@onNewAlvarMarkers: No marker in list")
            return
    foundMarker = None
    for marker in msg.markers:
        if marker.id == MARKER_ID:
            foundMarker = marker
            break
    if foundMarker is None:
        rospy.logwarn("catch_armarker.py@onNewAlvarMarkers: No matching marker in list")
        return
    rospy.loginfo("catch_armarker.py@onNewAlvarMarkers: Found marker with confidence: %d" % foundMarker.confidence)
    pose = foundMarker.pose  # geometry_msgs/PoseStamped

    # rotate to encounter the marker from the opposite position
    # http://answers.ros.org/question/69754/quaternion-transformations-in-python/
    quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    euler = transform.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    yaw -= math.pi/2.0
    quaternion = transform.quaternion_from_euler(roll, pitch, yaw)

    targetPose = foundMarker.pose
    targetPose.pose.position.x -= 2*X_DISTANCE
    targetPose.pose.orientation.x = quaternion[0]
    targetPose.pose.orientation.y = quaternion[1]
    targetPose.pose.orientation.z = quaternion[2]
    targetPose.pose.orientation.w = quaternion[3]

    pose.header.frame_id = FRAME_ID
    targetPose.header.frame_id = FRAME_ID

    if gripper.move(targetPose):
        targetPose.pose.position.x += X_DISTANCE
        if gripper.move(targetPose):
            targetPose.pose.position.x += X_DISTANCE
            if gripper.move(targetPose):
                gripper.grasp(GraspingMode.wide)
            else:
                rospy.logwarn("catch_armarker.py@onNewAlvarMarkers: Can't grasp")
        else:
            rospy.logwarn("catch_armarker.py@onNewAlvarMarkers: Can't move close to object")
    else:
        rospy.logwarn("catch_armarker.py@onNewAlvarMarkers: Can't move towards the object")


if __name__ == '__main__': main()