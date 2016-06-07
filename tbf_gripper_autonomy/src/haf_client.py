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

import os
import rospy
import actionlib

import geometry_msgs.msg
import haf_grasping.msg
import haf_grasping.srv
import sensor_msgs.msg

from visualization_msgs.msg import *


class HAFClient(object):

    def __init__(self):

        self.graspingcenter = geometry_msgs.msg.Point()
        self.graspingcenter.x = 0.0
        self.graspingcenter.y = 0.0
        self.graspingcenter.z = 0.0
        tmp_graspingcenter = rospy.get_param("grasp_search_center")
        if type(tmp_graspingcenter) == list and len(tmp_graspingcenter) == 3 and type(tmp_graspingcenter[0]) == float:
            self.graspingcenter.x = tmp_graspingcenter[0]
            self.graspingcenter.y = tmp_graspingcenter[1]
            self.graspingcenter.z = tmp_graspingcenter[2]

        self.approach_vector = geometry_msgs.msg.Vector3()
        self.approach_vector.x = 0.0
        self.approach_vector.y = 0.0
        self.approach_vector.z = 1.0
        tmp_approach_vector = rospy.get_param("gripper_approach_vector")
        if type(tmp_approach_vector) == list and len(tmp_approach_vector) == 3 and type(tmp_approach_vector[0]) == float:
            self.approach_vector.x = tmp_approach_vector[0]
            self.approach_vector.y = tmp_approach_vector[1]
            self.approach_vector.z = tmp_approach_vector[2]

        # max limit_x 32-14 = 18, limit_y = 44-14 = 30
        self.grasp_search_size_x = rospy.get_param("grasp_search_size_x", 18)
        self.grasp_search_size_y = rospy.get_param("grasp_search_size_y", 30)
        self.max_calculation_time = rospy.get_param("max_calculation_time", 50)
        self.grasp_calculation_time_max = rospy.Duration(self.max_calculation_time)
        self.show_only_best_grasp = rospy.get_param("show_only_best_grasp", True)
        self.base_frame_default = rospy.get_param("base_frame_default", "gripper_camera_rgb_frame")
        self.gripper_opening_width = rospy.get_param("gripper_width", 0.1)

        self.input_pc_topic = rospy.get_param("input_pc_topic", "/gripper_camera/depth_registered/points")
        self.pc_sub = rospy.Subscriber(self.input_pc_topic, sensor_msgs.msg.PointCloud2, callback=self.get_grasp_cb, queue_size=10)

        # services for setting parameters
        self.srv_set_grasp_center = rospy.Service("/haf_grasping/set_grasp_center", haf_grasping.srv.GraspSearchCenter, self.set_grasp_center)
        self.srv_set_grasp_search_area_size = rospy.Service("/haf_grasping/set_grasp_search_area_size", haf_grasping.srv.GraspSearchRectangleSize, self.set_grasp_search_area_size)
        self.srv_srv_set_grasp_calculation_time_max = rospy.Service("/haf_grasping/set_grasp_calculation_time_max", haf_grasping.srv.GraspCalculationTimeMax, self.set_grasp_calculation_time_max)
        self.srv_set_approach_vector = rospy.Service("/haf_grasping/set_approach_vector", haf_grasping.srv.GraspApproachVector, self.set_approach_vector)
        self.srv_set_show_only_best_grasp = rospy.Service("/haf_grasping/set_show_only_best_grasp", haf_grasping.srv.ShowOnlyBestGrasp, self.set_show_only_best_grasp)
        self.srv_set_gripper_width = rospy.Service("/haf_grasping/set_gripper_opening_width", haf_grasping.srv.GraspPreGripperOpeningWidth, self.set_gripper_width)

        # debug tools
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
        self.marker = Marker()
        self.marker.header.frame_id = self.base_frame_default
        self.marker.ns = "grasp_points"
        self.marker.id = 1
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD

        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.lifetime = rospy.Duration()

#Service Callbacks

    def get_grasp_cb(self, msg):
        rospy.loginfo("HAFClient.get_grasp_cb(): point cloud received")
        ac = actionlib.SimpleActionClient("calc_grasppoints_svm_action_server", haf_grasping.msg.CalcGraspPointsServerAction)
        rospy.loginfo("HAFClient.get_grasp_cb(): Waiting for action server to start.")
        ac.wait_for_server()
        rospy.loginfo("HAFClient.get_grasp_cb(): Action server started, sending goal.")
        goal = haf_grasping.msg.CalcGraspPointsServerGoal()

        goal.graspinput.input_pc = msg
        goal.graspinput.grasp_area_center = self.graspingcenter
        goal.graspinput.approach_vector = self.approach_vector
        goal.graspinput.grasp_area_length_x = self.grasp_search_size_x
        goal.graspinput.grasp_area_length_y = self.grasp_search_size_y
        goal.graspinput.max_calculation_time = self.grasp_calculation_time_max
        goal.graspinput.show_only_best_grasp = self.show_only_best_grasp
        goal.graspinput.gripper_opening_width = self.gripper_opening_width

        ac.send_goal(goal, done_cb=self.grasp_received_cb, active_cb=self.grasp_calculation_active_cb, feedback_cb=self.grasp_feedback_cb)

        finished_before_timeout = ac.wait_for_result(rospy.Duration(50.0))

        if finished_before_timeout:
            state = ac.get_state()
            result = ac.get_result()
            rospy.loginfo("HAFClient.get_grasp_cb(): Result: %s", result)
            rospy.loginfo("HAFClient.get_grasp_cb(): Action finished: %s", state)
        else:
            rospy.loginfo("HAFClient.get_grasp_cb(): Action did not finish before the time out.")

    # Parameter Services Callbacks
    def set_grasp_center(self, request):
        self.graspingcenter.x = request.graspserachcenter.x
        self.graspingcenter.y = request.graspserachcenter.y
        self.graspingcenter.z = request.graspserachcenter.z
        rospy.loginfo("HAFClient.set_grasp_center(): Set grasp search center to: x=%f, y=%f, z=%f",
                      request.graspserachcenter.x, request.graspserachcenter.y, request.graspserachcenter.z)
        result = haf_grasping.srv.GraspSearchCenterResponse()
        result.result = True
        rospy.loginfo("HAFClient.set_grasp_center(): sending back response: [%ld]", result.result)
        return result.result

    def set_grasp_search_area_size(self, request):
        self.grasp_search_size_x = request.grasp_search_size_x
        self.grasp_search_size_y = request.grasp_search_size_y
        rospy.loginfo("HAFClient.set_grasp_search_area_size(): Set grasp rectangle size to: x=%ld y=%ld",
                      request.grasp_search_size_x, request.grasp_search_size_y)
        result = haf_grasping.srv.GraspSearchRectangleSizeResponse()
        result.result = True
        rospy.loginfo("HAFClient.set_grasp_search_area_size(): sending back response: [%ld]", result.result)
        return result.result

    def set_grasp_calculation_time_max(self, request):
        self.grasp_calculation_time_max = request.max_calculation_time
        rospy.loginfo("HAFClient.set_grasp_calculation_time_max(): Set max calculation time (sec) to: x=%d",
                      request.max_calculation_time.secs)
        result = haf_grasping.srv.GraspCalculationTimeMaxResponse()
        rospy.loginfo("HAFClient.set_grasp_calculation_time_max(): sending back response: [%d]", result.result)
        result.result = True
        return result.result

    def set_approach_vector(self, request):
        self.approach_vector = request.approach_vector
        rospy.loginfo("HAFClient.set_approach_vector(): Set approach vector to: [%f,%f,%f]",
                      self.approach_vector.x, self.approach_vector.y, self.approach_vector.z)
        result = haf_grasping.srv.GraspApproachVectorResponse()
        result.result = True
        rospy.loginfo("HAFClient.set_approach_vector(): sending back response: [%d]", result.result)
        return result.result

    def set_show_only_best_grasp(self, request):
        self.show_only_best_grasp = request.show_only_best_grasp
        rospy.loginfo("HAFClient.set_show_only_best_grasp(): Set show_only_best_grasp to: [%d]",
                      self.show_only_best_grasp)
        result = haf_grasping.srv.ShowOnlyBestGraspResponse()
        result.result = True
        rospy.loginfo("HAFClient.set_show_only_best_grasp(): sending back response: [%d]", result.result)
        return result.result

    def set_gripper_width(self, request):
        self.gripper_opening_width = request.gripper_opening_width
        rospy.loginfo("HAFClient.set_gripper_width(): Set gripper_opening_width (factor for scaling pc) to: x=%d",
                      self.gripper_opening_width)
        result = haf_grasping.srv.GraspPreGripperOpeningWidthResponse()
        result.result = True
        rospy.loginfo("HAFClient.set_gripper_width(): sending back response: [%d]", result.result)
        return result.result

    # Grasp Calculation Action Server Callbacks
    def grasp_received_cb(self, state, result):
        rospy.loginfo("HAFClient.grasp_received_cb(): state = %d", state)
        rospy.loginfo("HAFClient.grasp_received_cb(): result.graspOutput.eval = %d", result.graspOutput.eval)
        rospy.loginfo("HAFClient.grasp_received_cb(): result.graspOutput.graspPoint1 = %s", result.graspOutput.graspPoint1)
        rospy.loginfo("HAFClient.grasp_received_cb(): result.graspOutput.graspPoint2 = %s", result.graspOutput.graspPoint2)
        rospy.loginfo("HAFClient.grasp_received_cb(): result.graspOutput.averagedGraspPoint = %s", result.graspOutput.averagedGraspPoint)
        rospy.loginfo("HAFClient.grasp_received_cb(): result.graspOutput.approachVector = %s", result.graspOutput.approachVector)
        rospy.loginfo("HAFClient.grasp_received_cb(): result.graspOutput.roll = %f", result.graspOutput.roll)
        self.marker.header.stamp = rospy.Time.now()
        self.marker.id = 1
        self.marker.pose.position.x = result.graspOutput.graspPoint1.x
        self.marker.pose.position.y = result.graspOutput.graspPoint1.y
        self.marker.pose.position.z = result.graspOutput.graspPoint1.z
        self.marker.pose.orientation.x = result.graspOutput.roll
        self.marker.color.b = 0.0
        self.marker_pub.publish(self.marker)
        self.marker.id = 2
        self.marker.pose.position.x = result.graspOutput.graspPoint2.x
        self.marker.pose.position.y = result.graspOutput.graspPoint2.y
        self.marker.pose.position.z = result.graspOutput.graspPoint2.z
        self.marker.color.b = 1.0
        self.marker_pub.publish(self.marker)
        self.marker.id = 3
        self.marker.pose.position.x = result.graspOutput.averagedGraspPoint.x
        self.marker.pose.position.y = result.graspOutput.averagedGraspPoint.y
        self.marker.pose.position.z = result.graspOutput.averagedGraspPoint.z
        self.marker.color.b = 0.5
        self.marker_pub.publish(self.marker)


    def grasp_calculation_active_cb(self):
        rospy.loginfo("HAFClient.grasp_calculation_active_cb(): alive")

    def grasp_feedback_cb(self, feedback):
        rospy.loginfo("HAFClient.grasp_feedback_cb(): feedback = %s", feedback.feedback)

    def changeMarker(self, feedback):
        rospy.loginfo("HAFClient.changeMarker(): marker changed")

if __name__ == '__main__':
    rospy.init_node("tubaf_haf_client", anonymous=False)
    obj = HAFClient()
    rospy.spin()