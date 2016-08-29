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
import tf
import copy
import random

import grasping.haf_client
import grasping.hand
import grasping.arm


from visualization_msgs.msg import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

"""@package grasping
This package gives is made to handle a grasping task. Assuming the object of interest is within vision of a defined
camera. The position of a grasp on this object is computed by the haf_grasping package and hand to a controller. This
then manages to grasp the object by making use of MoveIt! and a HandController.
@author: Steve Grehl
"""


class Controller(object):
    """
    Controller that handles a grasping/ pick up task. It does so by using a haf_graspin-Client, MoveIt-Wrapper and
    HandController. their functionalies are utilized to solve the grasping task.
    """
    # see: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html

    def __init__(self):
        """
        Default constructor that loads parameter from the parameter server, register callbacks and publisher
        """
        self.end_effector_link = rospy.get_param("end_effector_link", "gripper_robotiq_palm_planning")

        self.haf_client = grasping.haf_client.HAFClient()
        rospy.loginfo("controller.py: Controller(): initilized HAF client")

        self.haf_client.add_grasp_cb_function(self.grasp_at_pose)

        self.moveit_controller = grasping.arm.MoveItWrapper()
        rospy.loginfo("controller.py: Controller(): initilized arm")

        self.hand_controller = grasping.hand.HandController()
        rospy.loginfo("controller.py: Controller(): initilized hand")

        self.tf_listener = tf.TransformListener()

        self.haf_client.register_pc_callback()

        # Debugging & Development tools
        self.marker_id = 1
        self.marker_pub = rospy.Publisher("/cntrl_marker", Marker, queue_size=1)

        rospy.loginfo("controller.py: Controller(): finished initialization")

    def convert_grasp_pose(self, grasp_pose):
        """
        Converts a given pose from the camera to the end-effector frame, used for planing in MoveIt!
        :param grasp_pose: pose of the object to grasp in camera_frame
        :type grasp_pose: PoseStamped
        :return: grasping pose in end-effector frame (last element of the kinematic chain)
        :rtype: PoseStamped
        """
        ret_pose = copy.deepcopy(grasp_pose)
        ret_pose.header.frame_id = "base_link"
        now = rospy.Time.now()

        self.tf_listener.waitForTransform(grasp_pose.header.frame_id, ret_pose.header.frame_id, now, rospy.Duration(4))
        ret_pose = self.tf_listener.transformPose(ret_pose.header.frame_id, ps=grasp_pose)
        return ret_pose

    def show_marker(self, pose_stamped):
        """
        Visualize a marker at the given pose in rviz by publishing
        :param pose_stamped: pose where the marker should be published
        :type pose_stamped: PoseStamped
        :return: -
        :rtype: -
        """
        a_marker = Marker()
        a_marker.header = pose_stamped.header
        a_marker.pose = pose_stamped.pose
        a_marker.action = Marker.ADD
        a_marker.type = Marker.ARROW
        a_marker.scale.x = 0.12
        a_marker.scale.y = 0.06
        a_marker.scale.z = 0.03
        a_marker.color.r = random.random()
        a_marker.color.g = 0.5
        a_marker.color.b = 0.5
        a_marker.color.a = 1.0
        a_marker.id = self.marker_id
        self.marker_id += 1
        self.marker_pub.publish(a_marker)

    def grasp_at_pose(self, grasp_pose):
        """
        Callback function for a given grasp pose - Here is the logic and schedule stored, the main functionality is here
        :param grasp_pose: pose for the gripper in order to grasp the object
        :type grasp_pose: PoseStamped
        :return: -
        :rtype: -
        """
        # Pausing Recognition?
        # rospy.loginfo("controller.py: Controller.onGraspSearchCallback(): received grasp_pose")  # : %s", grasp_pose)
        self.haf_client.remove_grasp_cb_function(self.grasp_at_pose)
        self.haf_client.unregister_pc_callback()

        # Calculating Target and Hover Pose
        target_pose = self.convert_grasp_pose(grasp_pose)
        hover_pose = copy.deepcopy(target_pose)
        hover_pose.pose.position.z += 0.24  # double finger length

        # Show marker
        self.show_marker(target_pose)
        rospy.sleep(0.1)
        self.show_marker(hover_pose)

        # Ask user if grasp point is ok
        answer = raw_input("Controller: Use calculated grasp point? (y/n) ...")
        if answer == 'y':
            pass
        else:
            self.haf_client.register_pc_callback()
            self.haf_client.add_grasp_cb_function(self.grasp_at_pose)
            return

        self.hand_controller.openHand()
        rospy.sleep(1.0)
        origin = self.moveit_controller.get_current_joints()
        origin_pose = self.moveit_controller.get_current_pose(frame_id="gripper_ur5_base_link")

        # Move to Target
        rospy.loginfo("Controller.onGraspSearchCallback(): to hover_pose")
        if not self.move_to_pose(hover_pose, origin):
            self.haf_client.register_pc_callback()
            self.haf_client.add_grasp_cb_function(self.grasp_at_pose)
            return

        # rospy.sleep(3.0)
        rospy.loginfo("Controller.onGraspSearchCallback(): to target_pose")
        if not self.move_to_pose(target_pose, hover_pose):
            self.haf_client.register_pc_callback()
            self.haf_client.add_grasp_cb_function(self.grasp_at_pose)
            return

        # grasp object
        rospy.loginfo("Controller.onGraspSearchCallback(): closing hand")
        self.hand_controller.closeHand()
        rospy.sleep(3.0)  # wait till the hand grasp the object
        self.moveit_controller.grasped_object()

        # lift grasped object
        # plan path towards origin
        while not self.move_to_pose(origin_pose, target_pose):
            rospy.loginfo("Controller.onGraspSearchCallback(): try tp plan towards origin again ... ")
            rospy.sleep(0.5)

        hand_closed = True
        while hand_closed:
            answer = raw_input("Controller: Open Hand? (y/n) ...")
            if answer == 'y':
                self.hand_controller.openHand()
                # self.moveit_controller.remove_attached_object(box_name)
                rospy.loginfo("Controller.onGraspSearchCallback(): END")
                self.haf_client.register_pc_callback()
                self.haf_client.add_grasp_cb_function(self.grasp_at_pose)
                hand_closed = False
            else:
                 rospy.sleep(3.0)  # sleep so the hand can open
        self.hand_controller.restHand()

    def move_to_origin(self, origin):
        """
        Move the arm to the given joint set, that is assumed to be the origin of the gripper
        :param origin: joints
        :type origin: JointState
        :return: -
        :rtype: -
        """
        # plan path towards origin
        while not self.moveit_controller.plan_to_joints(origin):
            # no plan calculated
            answer = raw_input("Controller..move_to_pose(): couldn't calculate a plan - Plan again?"
                               " Clear Octomap and plan again? Abort? (y/c/n)")
            if answer == 'y' or answer == 'c':
                if answer == 'c':
                    rospy.wait_for_service("clear_octomap")
                    clear_octomap = rospy.ServiceProxy('clear_octomap', Empty)
                    clear_octomap()
            else:
                return False
        rospy.loginfo("Controller.move_to_origin(): Execution: Moving towards origin")
        # raw_input("Press any key to continue ...")
        executed = False
        try:
            while not rospy.is_shutdown() and not executed:
                executed = self.moveit_controller.move_to_pose()
                if executed:
                    rospy.loginfo("Controller.move_to_origin(): Execution: Moved to origin ")
                    # rospy.sleep(3.0)  # wait while the gripper moves towards the pose
                else:
                    # rospy.sleep(0.5)
                    rospy.loginfo("Controller.move_to_origin(): Execution: Try to move towards origin again")
        except:
            rospy.logerr("Controller.move_to_origin(): Error during Execution: PLaning to origin")
            if origin is not None:
                self.move_to_origin(origin)
            self.haf_client.register_pc_callback()
            self.haf_client.add_grasp_cb_function(self.grasp_at_pose)
            return False

    def move_to_pose(self, pose, origin):
        """
        Move the arm to a given pose, if any error occurs or no plan can be found hold the origin for recovery
        :param pose: desired pose of the end-effector (defined by the group in the srdf)
        :type pose: PoseStamped
        :param origin: joint states of the origin
        :type origin: JointState
        :return: return if the pose could be reached within the tolerances (after execution)
        :rtype: Boolean
        """
        # Planing
        rospy.loginfo("Controller.move_to_pose(): planning")
        while not self.moveit_controller.plan_to_pose(pose):
            # rospy.loginfo("Controller.move_to_pose(): couldn't calculate a plan")
            answer = raw_input("Controller..move_to_pose(): couldn't calculate a plan - Plan again?"
                               " Clear Octomap and plan again? Abort? (y/c/n)")
            if answer == 'y' or answer == 'c':
                if answer == 'c':
                    rospy.wait_for_service("clear_octomap")
                    clear_octomap = rospy.ServiceProxy('clear_octomap', Empty)
                    clear_octomap()
            else:
                if origin is not None:
                    self.move_to_pose(origin)
                return False
        # Executing
        executed = False
        rospy.loginfo("Controller.move_to_pose(): Execution: Moving towards pose ")
        try:
            while not rospy.is_shutdown() and not executed:
                executed = self.moveit_controller.move_to_pose()
                if executed:
                    rospy.loginfo("Controller.move_to_pose(): Execution: Moved to pose ")
                    # rospy.sleep(3.0)  # wait while the gripper moves towards the pose
                else:
                    # rospy.sleep(0.5)
                    rospy.loginfo("Controller.move_to_pose(): Execution: Try to move grasp pose again")
        except:
            rospy.logerr("Controller.move_to_pose(): Error during Execution: PLaning to origin")
            if origin is not None:
                self.move_to_pose(origin)
            return False

if __name__ == '__main__':
    rospy.init_node("tubaf_grasping_controller", anonymous=False)
    cntrl = Controller()
    rospy.spin()
