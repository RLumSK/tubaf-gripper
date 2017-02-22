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
import actionlib
import tf.transformations
import copy

import geometry_msgs.msg
import haf_grasping.msg
import haf_grasping.srv
import sensor_msgs.msg

from visualization_msgs.msg import *

"""@package grasping
This package gives is made to handle a grasping task. Assuming the object of interest is within vision of a defined
camera. The position of a grasp on this object is computed by the haf_grasping package and hand to a controller. This
then manages to grasp the object by making use of MoveIt! and a HandController.
@author: Steve Grehl
"""


class HAFClient(object):
    """
    Client for the haf_grasping action server
    """
    def __init__(self):
        """
        Default Constructor - Connect to the sever, Load parameters from the parameter server and pass them by using a
        set of services
        """
        self.action_server_name = rospy.get_param("~haf_server_name", "/haf_server")
        self.grasp_cbs = list()

        self.graspingcenter = geometry_msgs.msg.Point()
        self.graspingcenter.x = 0.0
        self.graspingcenter.y = 0.0
        self.graspingcenter.z = 0.0
        tmp_graspingcenter = rospy.get_param("~grasp_search_center", [0.0, 0.0, 0.0])
        if type(tmp_graspingcenter) == list and len(tmp_graspingcenter) == 3 and type(tmp_graspingcenter[0]) == float:
            self.graspingcenter.x = tmp_graspingcenter[0]
            self.graspingcenter.y = tmp_graspingcenter[1]
            self.graspingcenter.z = tmp_graspingcenter[2]

        self.approach_vector = geometry_msgs.msg.Vector3()
        self.approach_vector.x = 0.0
        self.approach_vector.y = 0.0
        self.approach_vector.z = 1.0
        tmp_approach_vector = rospy.get_param("~gripper_approach_vector", [0.0, 0.0, 1.0])
        if type(tmp_approach_vector) == list and len(tmp_approach_vector) == 3 and type(tmp_approach_vector[0]) == float:
            self.approach_vector.x = tmp_approach_vector[0]
            self.approach_vector.y = tmp_approach_vector[1]
            self.approach_vector.z = tmp_approach_vector[2]

        # max limit_x 32-14 = 18, limit_y = 44-14 = 30
        self.grasp_search_size_x = rospy.get_param("~grasp_search_size_x", 18)
        self.grasp_search_size_y = rospy.get_param("~grasp_search_size_y", 30)
        self.max_calculation_time = rospy.get_param("~max_calculation_time", 50)
        self.grasp_calculation_time_max = rospy.Duration(self.max_calculation_time)
        self.show_only_best_grasp = rospy.get_param("~show_only_best_grasp", True)
        self.base_frame_default = rospy.get_param("~base_frame_default", "gripper_camera_rgb_frame")
        self.gripper_opening_width = rospy.get_param("~gripper_width", 0.1)
        self.grasp_offset = rospy.get_param("~grasp_offset", 0.01)

        self.input_pc_topic = rospy.get_param("~input_pc_topic", "/gripper_camera/depth_registered/points")
        self.pc_sub = None

        # services for setting parameters
        self.srv_set_grasp_center = rospy.Service("/haf_grasping/set_grasp_center",
                                                  haf_grasping.srv.GraspSearchCenter,
                                                  self.set_grasp_center)
        self.srv_set_grasp_search_area_size = rospy.Service("/haf_grasping/set_grasp_search_area_size",
                                                            haf_grasping.srv.GraspSearchRectangleSize,
                                                            self.set_grasp_search_area_size)
        self.srv_srv_set_grasp_calculation_time_max = rospy.Service("/haf_grasping/set_grasp_calculation_time_max",
                                                                    haf_grasping.srv.GraspCalculationTimeMax,
                                                                    self.set_grasp_calculation_time_max)
        self.srv_set_approach_vector = rospy.Service("/haf_grasping/set_approach_vector",
                                                     haf_grasping.srv.GraspApproachVector,
                                                     self.set_approach_vector)
        self.srv_set_show_only_best_grasp = rospy.Service("/haf_grasping/set_show_only_best_grasp",
                                                          haf_grasping.srv.ShowOnlyBestGrasp,
                                                          self.set_show_only_best_grasp)
        self.srv_set_gripper_width = rospy.Service("/haf_grasping/set_gripper_opening_width",
                                                   haf_grasping.srv.GraspPreGripperOpeningWidth,
                                                   self.set_gripper_width)

        self.grasp_quality_threshold = rospy.get_param("~grasp_quality_threshold", 50)  # worst [-20, 99] best
        self.grasp_search_timeout = rospy.get_param("~grasp_search_timeout", 50.0)

        self.ac_haf = actionlib.SimpleActionClient(self.action_server_name,
                                                   haf_grasping.msg.CalcGraspPointsServerAction)

        # debug tools
        self.marker_array = MarkerArray()
        self.marker_array_pub = rospy.Publisher("grasp_marker_array", MarkerArray, queue_size=1)

        self.marker = Marker()
        self.marker_pub = rospy.Publisher("grasp_marker", Marker, queue_size=1)

        self.marker.id = 1
        self.marker.type = Marker.ARROW
        # self.marker.type = Marker.MESH_RESOURCE
        # self.marker.mesh_resource = "package://robotiq_s_model_visualization/meshes/s-model_articulated/visual/full_hand.stl"
        self.marker_frame_id = "/gripper_robotiq_palm"

        self.marker.scale.x = 0.15
        self.marker.scale.y = 0.06
        self.marker.scale.z = 0.03
        # self.marker.scale.x = 1.
        # self.marker.scale.y = 1.
        # self.marker.scale.z = 1.

        self.marker.color.r = 0.1
        self.marker.color.g = 0.1
        self.marker.color.b = 0.1
        self.marker.color.a = 0.75

        self.rate = rospy.Rate(10)

    def register_pc_callback(self):
        """
        Subscribe to a point cloud topic
        :return: -
        :rtype: -
        """
        self.input_pc_topic = rospy.get_param("~input_pc_topic", self.input_pc_topic)
        self.pc_sub = rospy.Subscriber(self.input_pc_topic, sensor_msgs.msg.PointCloud2, callback=self.get_grasp_cb,
                                       queue_size=1)
        self.rate = rospy.Rate(1)
        rospy.loginfo("HAFClient.register_pc_callback(): subscribed to pointcloud topic: %s", self.input_pc_topic)

    def unregister_pc_callback(self):
        """
        unsubscribe from the point cloud
        :return: -
        :rtype: -
        """
        self.pc_sub.unregister()
        rospy.loginfo("HAFClient.unregister_pc_callback(): unsubscribed from pointcloud topic: %s", self.input_pc_topic)

    def set_graspingcenter(self, x=0, y=0, z=0):
        """
        set the grasping search center for the HAF algorithm
        :param x: x-value
        :type x: double
        :param y: y-value
        :type y: double
        :param z: z-value
        :type z: double
        :return: -
        :rtype: -
        """
        self.graspingcenter.x = x
        self.graspingcenter.y = y
        self.graspingcenter.z = z

    def increment_set_graspingcenter(self):
        """
        increments the x-value of the grasp search center for the HAF algorithm
        :return: -
        :rtype: -
        """
        if self.graspingcenter.x > rospy.get_param("~grasp_search_max", 0.6):
            tmp_graspingcenter = rospy.get_param("~grasp_search_center", [0.0, 0.0, 0.0])
            self.graspingcenter.x = tmp_graspingcenter[0]
        else:
            self.graspingcenter.x += rospy.get_param("~grasp_search_inc", 0.05)

    # Service Callbacks
    def get_grasp_cb(self, msg):
        """
        Query the action server for a grasp point using the given point cloud
        :param msg: a point cloud
        :type msg: PointCloud2
        :return: -
        :rtype: -
        """
        # unsubscribe point cloud msges to avoid flooding
        self.unregister_pc_callback()
        rospy.logdebug("HAFClient.get_grasp_cb(): point cloud received")
        rospy.logdebug("HAFClient.get_grasp_cb(): Waiting for action server to start.")
        self.ac_haf.wait_for_server()
        rospy.logdebug("HAFClient.get_grasp_cb(): Action server started, sending goal.")
        goal = haf_grasping.msg.CalcGraspPointsServerGoal()
        goal.graspinput.input_pc = msg
        goal.graspinput.grasp_area_center = self.graspingcenter
        goal.graspinput.approach_vector = self.approach_vector
        goal.graspinput.grasp_area_length_x = self.grasp_search_size_x
        goal.graspinput.grasp_area_length_y = self.grasp_search_size_y
        goal.graspinput.max_calculation_time = self.grasp_calculation_time_max
        goal.graspinput.show_only_best_grasp = self.show_only_best_grasp
        goal.graspinput.gripper_opening_width = self.gripper_opening_width
        goal.graspinput.goal_frame_id = self.base_frame_default# msg.header.frame_id

        self.ac_haf.send_goal(goal, done_cb=self.grasp_received_cb, active_cb=self.grasp_calculation_active_cb,
                              feedback_cb=self.grasp_feedback_cb)

        finished_before_timeout = self.ac_haf.wait_for_result(rospy.Duration(self.grasp_search_timeout))

        if finished_before_timeout:
            # state = self.ac_haf.get_state()
            result = self.ac_haf.get_result()
            rospy.logdebug("HAFClient.get_grasp_cb(): Action finished - Result: %s", result)
            if result.graspOutput.eval <= -20:
                rospy.logwarn("HAFClient.get_grasp_cb(): Worst quality of the estimated grasp point (" +
                              str(result.graspOutput.eval)+")")
                if self.graspingcenter.x > 1.0:
                    self.graspingcenter.x = 0.4
                else:
                    self.graspingcenter.x += 0.025
            else:
                # found grasp
                rospy.loginfo("HAFClient.get_grasp_cb(): Found valid grasp.")
                # continue grasping task
                return

        else:
            rospy.logdebug("HAFClient.get_grasp_cb(): Action did not finish before the time out.")
        # self.ac_haf.cancel_all_goals()
        # self.rate.sleep()
        # # no valid grasp found -> continue grasp search
        # self.register_pc_callback()

    # Parameter Services Callbacks
    def set_grasp_center(self, request):
        self.graspingcenter.x = request.graspsearchcenter.x
        self.graspingcenter.y = request.graspsearchcenter.y
        self.graspingcenter.z = request.graspsearchcenter.z
        rospy.loginfo("HAFClient.set_grasp_center(): Set grasp search center to: x=%f, y=%f, z=%f",
                      request.graspsearchcenter.x, request.graspsearchcenter.y, request.graspsearchcenter.z)
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

    # event managment
    def add_grasp_cb_function(self, function):
        self.grasp_cbs.append(function)

    def remove_grasp_cb_function(self, function):
        self.grasp_cbs.remove(function)

    def purge_grasp_cb_functions(self):
        self.grasp_cbs = list()

    @staticmethod
    def logresult(state, result):
        rospy.loginfo("HAFClient.logresult(): state = %d", state)
        rospy.loginfo("HAFClient.logresult(): result.graspOutput.eval = %d", result.graspOutput.eval)
        rospy.loginfo("HAFClient.logresult(): result.graspOutput.graspPoint1 = %s",
                      result.graspOutput.graspPoint1)
        rospy.loginfo("HAFClient.logresult(): result.graspOutput.graspPoint2 = %s",
                      result.graspOutput.graspPoint2)
        rospy.loginfo("HAFClient.logresult(): result.graspOutput.averagedGraspPoint = %s",
                      result.graspOutput.averagedGraspPoint)
        rospy.loginfo("HAFClient.logresult(): result.graspOutput.approachVector = %s",
                      result.graspOutput.approachVector)
        rospy.loginfo("HAFClient.logresult(): result.graspOutput.roll = %f", result.graspOutput.roll)

    def evaluate_grasp_result(self, result):
        # see: http://wiki.ros.org/haf_grasping -> 3.4 RESULT (roll in [radians])
        # result.graspOutput: 55     0.07 0.08 0.158921   0.07 0.02 0.158921     0 0 1     0.07 0.05 0.158921 90
        #                      |      |     |    |          |    |      |        | | |      |     |     |      |
        #                     val gp1(x,    y,   z)     gp2(x,   y,     z )   av(x,y,z) gcp(x,    y,    z)    roll
        self.grasp_quality_threshold = rospy.get_param("~grasp_quality_threshold", self.grasp_quality_threshold)  # worst [-20, 99] best
        if result.graspOutput.eval >= self.grasp_quality_threshold:
            rospy.logdebug("HAFClient.evaluate_grasp_result(): Grasp good enough %f > %f",
                           result.graspOutput.eval, self.grasp_quality_threshold)
            return True
        else:
            rospy.logdebug("HAFClient.evaluate_grasp_result(): Grasp good not enough %f < %f",
                           result.graspOutput.eval, self.grasp_quality_threshold)
            return False

    # Grasp Calculation Action Server Callbacks
    def grasp_received_cb(self, state, result):
        HAFClient.logresult(state, result)
        if self.evaluate_grasp_result(result):
            del self.marker_array.markers[:]
            approach_vec = [result.graspOutput.approachVector.x, result.graspOutput.approachVector.y,
                            result.graspOutput.approachVector.z]
            rospy.logdebug("HAFClient.grasp_received_cb(): approach_vec: %s", approach_vec)
            position_vec = [result.graspOutput.averagedGraspPoint.x, result.graspOutput.averagedGraspPoint.y,
                            result.graspOutput.averagedGraspPoint.z]

            orientation = tf.transformations.quaternion_about_axis(1*result.graspOutput.roll, approach_vec) # +numpy.pi/4.0
            quaternion = geometry_msgs.msg.Quaternion(*orientation)

            norm_approach = [element / tf.transformations.vector_norm(approach_vec) for element in approach_vec]

            hdr = std_msgs.msg.Header()
            hdr.stamp = rospy.Time.now()
            hdr.frame_id = result.graspOutput.header.frame_id

            grasp_pose = geometry_msgs.msg.PoseStamped(header=hdr)
            grasp_pose.pose.position.x = position_vec[0] + norm_approach[0] * self.grasp_offset
            grasp_pose.pose.position.y = position_vec[1] + norm_approach[1] * self.grasp_offset
            grasp_pose.pose.position.z = position_vec[2] + norm_approach[2] * self.grasp_offset
            grasp_pose.pose.orientation = quaternion

            # visualize marker for debugging and development
            m3 = Marker(header=hdr, id=2, scale=geometry_msgs.msg.Vector3(0.05, 0.01, 0.005))
            m3.pose = copy.deepcopy(grasp_pose.pose)
            m3.pose.position.x = position_vec[0]
            m3.pose.position.y = position_vec[1]
            m3.pose.position.z = position_vec[2]
            m3.color.r = 0.
            m3.color.g = 0.
            m3.color.b = 1.
            m3.color.a = 1.

            # self.marker_array.markers.append(m3)
            # self.marker_array_pub.publish(self.marker_array)

            # o2 = tf.transformations.quaternion_about_axis(-1.0*numpy.pi/2.0, [0, 0, 1])

            self.marker.pose = m3.pose
            # self.marker.pose.orientation = geometry_msgs.msg.Quaternion(
            #     *tf.transformations.quaternion_multiply(orientation, o2))
            self.marker.header = m3.header
            self.marker_pub.publish(self.marker)

            if state == actionlib.GoalStatus.SUCCEEDED:
                for function in self.grasp_cbs:
                    rospy.logdebug("haf_client.py:HAFClient.grasp_received_cb(): passing poses")
                    function(grasp_pose)
                pass
        else:
            # grasp is not good enough
            rospy.logdebug("HAFClient.grasp_received_cb(): Grasp not good enough - register point cloud callback")
            self.register_pc_callback()

    def grasp_calculation_active_cb(self):
        rospy.logdebug("HAFClient.grasp_calculation_active_cb(): alive")
        pass

    def grasp_feedback_cb(self, feedback):
        rospy.logdebug("HAFClient.grasp_feedback_cb(): %s", feedback)
        pass

    def changeMarker(self, feedback):
        rospy.logdebug("HAFClient.changeMarker(): marker changed")

# helper

def generate_grasp_marker(p, o, m1, m2):
    import numpy as np
    pos = np.array(p)
    rot = tf.transformations.quaternion_matrix(o)
    scale = 0.1
    v1 = (0, scale,0, 1)
    v2 = (0, -scale,0, 1)

    var1 = np.dot(rot,v1)
    var2 = np.dot(rot,v2)

    a1 = pos+(var1[0:3])/var1[3]
    a2 = pos+(var2[0:3])/var2[3]

    # print rot
    # print p, o, pos, var1, var2, a1, a2
    m1.color.r = 0.1
    m1.color.g = 0.1
    m1.color.b = 0.1
    m1.color.a = 1.0

    m2.color.r = 0.0
    m2.color.g = 0.0
    m2.color.b = 0.0
    m2.color.a = 1.0

    m1.pose.position = geometry_msgs.msg.Point(*a1)
    m1.scale = geometry_msgs.msg.Vector3(0.1, 0.01, 0.005)

    m2.pose.position = geometry_msgs.msg.Point(*a2)
    m2.scale = geometry_msgs.msg.Vector3(0.1, 0.01, 0.005)

    return [m1, m2]


if __name__ == '__main__':
    rospy.init_node("tubaf_haf_client", anonymous=False)
    obj = HAFClient()
    obj.register_pc_callback()

    rospy.spin()
