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
import roslib

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as transform
import ar_track_alvar_msgs.msg as ar_msg


from tbf_gripper_rqt import gripper_module

MARKER_ID = 42

class ARCatcher:
    def __init__(self):
        ## Setup ROS
        rospy.loginfo("ARCatcher.py: Starting init")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ar_catcher', anonymous=False)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.display_target_pose_publisher = rospy.Publisher('/target_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

        self.subscriber = rospy.Subscriber("/ar_pose_marker", ar_msg.AlvarMarkers, self.onNewAlvarMarkers)

        ## Setup robot controller
        self.hand = gripper_module.WideGripperModel()
        self.gripper = moveit_commander.RobotCommander()  # This object is an interface to the robot as a whole.
        self.scene = moveit_commander.PlanningSceneInterface()  # This object is an interface to the world surrounding the robot.
        self.ur5 = moveit_commander.MoveGroupCommander("arm")  # This interface can be used to plan and execute motions on the arm.


    def onNewAlvarMarkers(self, msg):
        if len(msg.markers) == 0:
            rospy.logwarn("ARCatcher.py@ARCatcher.onNewAlvarMarkers: No marker in list")
            return
        foundMarker = None
        for marker in msg.markers:
            if marker.id == MARKER_ID:
                foundMarker = marker
                break
        if foundMarker is None:
            rospy.logwarn("ARCatcher.py@ARCatcher.onNewAlvarMarkers: No matching marker in list")
            return
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Found marker with confidence: %d" % foundMarker.confidence)
        pose = foundMarker.pose  # geometry_msgs/PoseStamped

        # rotate to encounter the marker from the opposite position
        # http://answers.ros.org/question/69754/quaternion-transformations-in-python/
        quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        euler = transform.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        # roll = roll + math.pi/2.0
        # pitch = pitch + math.pi/2.0
        yaw = yaw - math.pi/2.0

        quaternion = transform.quaternion_from_euler(roll, pitch, yaw)
        targetPose = foundMarker.pose
        targetPose.pose.position.x = targetPose.pose.position.x/2.0
        targetPose.pose.orientation.x = quaternion[0]
        targetPose.pose.orientation.y = quaternion[1]
        targetPose.pose.orientation.z = quaternion[2]
        targetPose.pose.orientation.w = quaternion[3]

        pose.header.frame_id = "camera_link"
        targetPose.header.frame_id = "camera_link"

        self.display_target_pose_publisher.publish(targetPose)
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Generating a plan")
        self.ur5.set_pose_target(targetPose)

        plan = self.ur5.plan()
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Plan: %s" % plan)

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = self.gripper.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Start moving")
        ## Moving to a pose goal
        self.ur5.go(wait=True)
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Stopped moving?")
        self.subscriber.unregister()
        pause = 10.
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Unregistered subsciber - sleeping %d" % pause)
        rospy.sleep(pause)
        self.subscriber = rospy.Subscriber("/ar_pose_marker", ar_msg.AlvarMarkers, self.onNewAlvarMarkers)
        rospy.loginfo("ARCatcher.py@ARCatcher.onNewAlvarMarkers: Registered subsciber")

def main():
    """
    Start a ARCatcher-instance and keep it running via rospy.spin()
    @return:
    """
    try:
        ar_catcher = ARCatcher()
        rospy.spin()
    except Exception, e:
        print("An errror occured: "+str(e))


if __name__ == '__main__': main()
