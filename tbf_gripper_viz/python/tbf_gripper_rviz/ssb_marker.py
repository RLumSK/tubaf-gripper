#!/usr/bin/python
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

from __future__ import print_function, division

"""
@package ssb_marker
This package stores interactive Marker for the smart sensor boxes developed in the ARIDuA project.
The markers are used to support the place task of a manipulator.
General information about creating interactive marker can be found in the tutorials.
http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started#simple_marker
https://github.com/ros-visualization/visualization_tutorials/blob/kinetic-devel/interactive_marker_tutorials/scripts/simple_marker.py
"""

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from interactive_markers.menu_handler import *

from message_filters import Cache, Subscriber
from geometry_msgs.msg import QuaternionStamped, Pose, PoseStamped

from tf import TransformListener, transformations
from numpy import pi

from tbf_gripper_tools.SmartEquipment import SmartEquipment
# from tubaf_tools.help import invert_pose, add_pose
import os


# https://www.tutorialspoint.com/python_design_patterns/python_design_patterns_singleton.htm
class MarkerServerSingleton:
    __instance = None

    @staticmethod
    def get_instance():
        """ Static access method. """
        if MarkerServerSingleton.__instance is None:
            MarkerServerSingleton()
        return MarkerServerSingleton.__instance

    def __init__(self):
        """ Virtually private constructor. """
        if MarkerServerSingleton.__instance is not None:
            raise Exception("[MarkerServerSingleton] This class is a singleton!")
        else:
            MarkerServerSingleton.__instance = InteractiveMarkerServer(rospy.get_param("interactive_marker_server_name",
                                                                                       "SE_MarkerServer"))


class SSBMarker(InteractiveMarker):
    """
    Class for the smart sensor box interactive marker
    """

    def __init__(self, name="~", pose=None, mesh=None, controls="xyr"):
        """
        Default constructor
        @:param name: name of this marker
        @:type name: str
        @:param pose: initial pose of the marker
        @:type pose: geometry_msgs.msg.PoseStamped
        """
        super(SSBMarker, self).__init__()

        # ROS
        self.pose_topic = "/ssb_pose"
        self._pub_pose_stamped = rospy.Publisher(name=self.pose_topic, data_class=geometry_msgs.msg.PoseStamped,
                                                 queue_size=5)
        self._normal_cache = Cache(Subscriber("/ork/floor_normal", QuaternionStamped), 10, allow_headerless=False)
        self.tf_listener = TransformListener()
        rospy.sleep(0.2)
        # Marker
        self._reference_frame = None

        self.name = name
        self.description = "[" + self.name + "] Marker"
        self.scale = rospy.get_param(name + "scale", 0.5)

        if pose is None:
            pose = geometry_msgs.msg.PoseStamped()
            self._reference_frame = "base_footprint"
        else:
            self._reference_frame = pose.header.frame_id
        # rospy.loginfo(pose)
        self.pose = pose.pose
        self.header.frame_id = self._reference_frame

        self._mesh_marker = Marker()
        self._mesh_marker.type = Marker.MESH_RESOURCE
        if mesh is None:
            mesh = 'package://tbf_gripper_tools/resources/mesh/old_ssb.stl'
        self._mesh_marker.mesh_resource = mesh
        self._mesh_marker.scale.x = 1.0
        self._mesh_marker.scale.y = 1.0
        self._mesh_marker.scale.z = 1.0
        self._mesh_marker.color.r = 0.0
        self._mesh_marker.color.g = 0.0
        self._mesh_marker.color.b = 1.0
        self._mesh_marker.color.a = 1.0

        self._mesh_cntrl = InteractiveMarkerControl()
        self._mesh_cntrl.always_visible = True
        self._mesh_cntrl.markers.append(self._mesh_marker)
        self._mesh_cntrl.interaction_mode = InteractiveMarkerControl.MENU
        self.controls.append(self._mesh_cntrl)

        if "x" in controls:
            self._move_x_cntrl = InteractiveMarkerControl()
            self._move_x_cntrl.name = "move_x"
            self._move_x_cntrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            self.controls.append(self._move_x_cntrl)
        if "y" in controls:
            self._move_y_cntrl = InteractiveMarkerControl()
            self._move_y_cntrl.name = "move_y"
            self._move_y_cntrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            # see: https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
            self._move_y_cntrl.orientation.w = 0.5
            self._move_y_cntrl.orientation.x = -0.5
            self._move_y_cntrl.orientation.y = -0.5
            self._move_y_cntrl.orientation.z = 0.5
            self.controls.append(self._move_y_cntrl)
        if "r" in controls:
            self._rot_cntrl = InteractiveMarkerControl()
            self._rot_cntrl.name = "rotate"
            self._rot_cntrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS  # MOVE_ROTATE
            # see: https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
            self._rot_cntrl.orientation.w = 0.7071
            self._rot_cntrl.orientation.x = 0
            self._rot_cntrl.orientation.y = 0.7071
            self._rot_cntrl.orientation.z = 0
            self.controls.append(self._rot_cntrl)

        # https://github.com/RobotnikAutomation/robotnik_purepursuit_planner/blob/master/robotnik_pp_planner/scripts/robotnik_pp_planner/path_marker.py
        self._menu_handler = MenuHandler()
        self._menu_handler.insert("Update Normal", callback=self.onUpdateNormal)
        self._menu_handler.insert("Publish Pose", callback=self.onPublishPose)

        # rospy.logdebug("ssb_marker.SSBMarker(): Header: %s", self.header)
        # rospy.logdebug("ssb_marker.SSBMarker(): Pose of the Marker is\n%s", self.pose)
        # rospy.logdebug("ssb_marker.SSBMarker(): Mesh: %s", mesh)
        # rospy.logdebug("ssb_marker.SSBMarker(): Initialized")

    @classmethod
    def from_SmartEquipment(cls, se, marker_pose):
        """
        Constructor using Smart Equipment
        :param marker_pose: pose where the marker should appear
        :type marker_pose: PoseStamped
        :param se: equipment
        :type se: SmartEquipment
        """
        return cls(name=se.name, pose=marker_pose, mesh=os.path.join("package://" + se.mesh_pkg, se.mesh_rel_path))

    def getMeshResourcePath(self):
        """
        Return the location of the used mesh
        :return: OS path
        :rtype: String
        """
        return self._mesh_marker.mesh_resource

    def getMeshScale(self):
        """
        Return the scale of the used mesh
        :return: OS path
        :rtype: String
        """
        return [self._mesh_marker.scale.x, self._mesh_marker.scale.y, self._mesh_marker.scale.z]

    def onFeedback(self, feedback):
        """
        Callback from the Interactive Marker Server
        :param feedback: Feedback from the GUI
        :type feedback: visualization_msgs.msg.InteractiveMarkerFeedback
        :return: -
        :rtype: -
        """
        # rospy.logdebug("SSBMarker.onFeedback(): %s" % feedback)
        self.pose = feedback.pose

    def onPublishPose(self, feedback):
        """
        Callback from the Interactive Marker Server
        :param feedback: Feedback from the GUI
        :type feedback: visualization_msgs.msg.InteractiveMarkerFeedback
        :return: -
        :rtype: -
        """
        self.onFeedback(feedback)
        ps = PoseStamped()
        ps.header = feedback.header
        ps.header.seq = 0
        ps.header.stamp = rospy.Time.now()
        ps.pose = self.pose  # pose is updated by onFeedback
        # rospy.logdebug("SSBMarker.onPublishPose(): \n%s", ps)
        self._pub_pose_stamped.publish(ps)

    def onUpdateNormal(self, feedback):
        """
        Updating the normal from floor detection
        :param feedback: Feedback from the GUI
        :type feedback: visualization_msgs.msg.InteractiveMarkerFeedback
        :return: -
        :rtype: -
        """
        self.onFeedback(feedback)
        transform_available = False
        target = self._reference_frame
        t = feedback.header.stamp
        while not transform_available:
            qs = self._normal_cache.getLast()
            source = qs.header.frame_id
            transform_available = self.tf_listener.canTransform(target_frame=target, source_frame=source, time=t)
            # rospy.logdebug("SSBMarker.onUpdateNormal(): canTransform from '%s' to '%s' at time '%s'? %r" %
            #               (target, source, t, transform_available))
            rospy.sleep(0.2)
        qs = self.tf_listener.transformQuaternion(self._reference_frame, qs)
        server = MarkerServerSingleton.get_instance()
        pose = Pose()
        pose.position = server.get(self.name).pose.position
        pose.orientation = qs.quaternion
        server.setPose(self.name, pose, header=qs.header)
        rospy.loginfo("SSBMarker.onUpdateNormal(): New Pose is %s" % pose)
        server.applyChanges()

    def add_custom_menu_entry(self, menu_entry, callback, parent=None):
        """
        Adds a new menu entry underneath the specified parent
        :param menu_entry: new menu entry for the drop-down-menu
        :type menu_entry: str
        :param callback: callback function
        :type callback: function
        :param parent: entry above the new one
        :type parent: handle
        :return: menu handle
        :rtype: handle
        """
        handle = self._added_menu_entry_before(menu_entry, parent)
        if handle is None:
            handle = self._menu_handler.insert(menu_entry, parent=parent, callback=callback)
            self._menu_handler.reApply(MarkerServerSingleton.get_instance())
        return handle

    def _added_menu_entry_before(self, menu_entry, parent):
        """
        Checks if the menu_entry under parent exists already
        :param menu_entry:
        :type menu_entry: str
        :param parent: handle of the parent in the menu_handler
        :type parent: handle
        :return: handle of the entry or None
        :rtype: handle or None
        """
        for pair in self._menu_handler.entry_contexts_.items():
            print(pair[1].title)
            if pair[1].title == menu_entry:
                return pair[0]
        return None

    def get_pose_topic(self):
        """
        Return the topic, where a pose is expected to be published
        :return: pose topic
        :rtype: str
        """
        return self.pose_topic

    def enable_marker(self):
        """
        Visualize the marker
        :return: -
        :rtype: -
        """
        MarkerServerSingleton.get_instance().insert(self, self.onFeedback,
                                                    feedback_type=InteractiveMarkerServer.DEFAULT_FEEDBACK_CB)
        # MarkerServerSingleton.get_instance().insert(self, self.onUpdateNormal)
        self._menu_handler.apply(MarkerServerSingleton.get_instance(), self.name)
        MarkerServerSingleton.get_instance().applyChanges()

    def disable_marker(self):
        """
        Disable/Hide the marker
        :return: -
        :rtype: -
        """
        MarkerServerSingleton.get_instance().clear()
        MarkerServerSingleton.get_instance().applyChanges()


class SSBGraspedMarker(SSBMarker):
    """
    Add a Hand to the Station to visualize a grasped SSB
    """

    def __init__(self, name="~", pose=None, mesh=None, gripper_pose=None, ns="~", controls="xyr"):
        super(SSBGraspedMarker, self).__init__(name=name, pose=pose, mesh=mesh, controls=controls)

        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = rospy.get_param(ns + "gripper_mesh_resource",
                   'package://robotiq_3f_gripper_visualization/meshes/robotiq-3f-gripper/visual/GRIPPER_CLOSED.stl')
        gripper_marker.scale = self._mesh_marker.scale
        if gripper_pose is None:
            gripper_marker.pose = self._mesh_marker.pose
            rospy.logwarn("SSBGraspedMarker.__init__() No gripper_pose given setting it to the equipment pose - "
                          "to use equipment withoput gripper please use SSBMarker instead")
        else:
            gripper_marker.pose = gripper_pose.pose
            gripper_marker.header = gripper_pose.header
        rospy.loginfo("SSBGraspedMarker.__init__() gripper_maker.pose %s" % gripper_marker.pose)

        gripper_marker.color.r = 0.0
        gripper_marker.color.g = 1.0
        gripper_marker.color.b = 0.0
        gripper_marker.color.a = 1.0

        # Create an empty control
        gripper_control = InteractiveMarkerControl()
        gripper_control.always_visible = True
        # gripper_control.name = self.name + "_gripper"
        # gripper_control.interaction_mode = InteractiveMarkerControl.NONE
        # gripper_control.orientation_mode = InteractiveMarkerControl.FIXED
        gripper_control.markers.append(gripper_marker)

        self.controls.append(gripper_control)
        self.gripper_pose = gripper_pose

    @classmethod
    def from_SmartEquipment(cls, se, marker_pose, tf_listener=None, save_relation=False, use_relation=False):
        """
        Constructor using Smart Equipment
        :param marker_pose: pose where the marker should appear
        :type marker_pose: PoseStamped
        :param use_relation: use the ssb-gripper relation
        :param save_relation: save the ssb-gripper relation
        :param tf_listener: Transform listener should only run once per application
        :param se: equipment
        :type se: SmartEquipment
        """
        # rospy.logdebug("%s", se)
        if tf_listener is None:
            tf_listener = TransformListener()
            rospy.sleep(2.0)
        rospy.loginfo(se.name)
        return cls(name=se.name, pose=marker_pose, mesh=os.path.join("package://" + se.mesh_pkg, se.mesh_rel_path),
                   gripper_pose=se.get_grasp_pose(object_pose_stamped=marker_pose,tf_listener=tf_listener,
                                                  save_relation=save_relation, use_relation=use_relation),
                   ns="~", controls="")


class SSBGraspMarker(object):
    """
    Define the Grasp Point on the SSB
    """

    def __init__(self, ssb_marker, ns="~"):
        """
        Default constructor
        @:param ssb_marker: parent marker
        @:type ssb_marker: SSBMarker
        @:param ns: number of this ssb marker
        @:type ns: str
        """
        self.ssb_marker = ssb_marker
        ssb_marker.add_custom_menu_entry("Select Grasp here", self.onGraspSelect)
        self.grasp_topic = rospy.get_param(ns + "grasp_topic", "/ssb_grasp_pose")
        self.pub_GraspPose = None
        self.pose = None

        # self.scale = ssb_marker.scale * 0.75
        self.ns = ns
        # Distance between origin and grasp point
        self.gripper_offset = -0.075

    def onGraspSelect(self, feedback):
        """
        Publish current mouse position on the smart sensor box as grasp pose
        :param feedback: Feedback from the GUI
        :type feedback: visualization_msgs.msg.InteractiveMarkerFeedback
        :return:
        :rtype:
        """
        self.pub_GraspPose = rospy.Publisher(name=self.grasp_topic, data_class=geometry_msgs.msg.PoseStamped,
                                             queue_size=5)
        int_marker = self._generate_gripper(feedback.mouse_point)
        self.pose = int_marker.pose
        self.pose.orientation = feedback.pose.orientation
        server = self.ssb_marker.get_server()
        server.insert(int_marker, self.onOrientationFeedback)
        menu_handle = self.ssb_marker.add_custom_menu_entry("Grasping", callback=self.onOrientationFeedback)
        self.ssb_marker.add_custom_menu_entry("Grasp Here", callback=self.onGraspPose, parent=menu_handle)
        server.applyChanges()
        rospy.sleep(0.2)

    def onGraspPose(self, feedback):
        """
        Publish current mouse position on the smart sensor box as grasp pose
        :param feedback: Feedback from the GUI
        :type feedback: visualization_msgs.msg.InteractiveMarkerFeedback
        :return: -
        :rtype: -
        """
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = feedback.header.frame_id
        grasp_pose.pose = self.pose
        # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
        rot_z_90 = transformations.quaternion_about_axis(pi / 2.0, [0, 0, 1])
        q = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        res = transformations.quaternion_multiply(q, rot_z_90)
        grasp_pose.pose.orientation.w = res[3]
        grasp_pose.pose.orientation.x = res[0]
        grasp_pose.pose.orientation.y = res[1]
        grasp_pose.pose.orientation.z = res[2]
        self.pub_GraspPose.publish(grasp_pose)
        # rospy.logdebug("SSBGraspMarker.onGraspPose(): Grasp Pose is %s" % grasp_pose)

    def onOrientationFeedback(self, feedback):
        """
        Feedback from the gripper marker
        :param feedback: Feedback from the GUI
        :type feedback: visualization_msgs.msg.InteractiveMarkerFeedback
        :return: -
        :rtype: -
        """
        # rospy.logdebug("SSBGraspMarker.onOrientationFeedback(): %s" % feedback)
        self.pose = feedback.pose

    def _generate_gripper(self, offset):
        """
        Spawn a gripper marker on the determined position to query the orientation
        :param offset: relative position on the marker
        :type offset: geometry_msgs.msg.Point
        :return: interactive gripper marker
        :rtype: InteractiveMarker
        """
        ns = self.ns
        factor = 0.75
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.ssb_marker.header.frame_id
        int_marker.pose.position.x = offset.x
        int_marker.pose.position.y = offset.y
        int_marker.pose.position.z = offset.z
        int_marker.scale = factor * self.ssb_marker.scale
        int_marker.name = self.ssb_marker.name + "_gripper"
        int_marker.header = self.ssb_marker.header
        int_marker.description = "[" + ns + "] Grasp Marker"

        # Generate hand as marker and add it to the interactive marker as control
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = rospy.get_param(ns + "ssb_mesh_resource",
                                               'package://robotiq_s_model_visualization/meshes/s-model_articulated/visual/full_hand.stl')
        # marker.scale = factor * self.ssb_marker.scale
        marker.pose.position.y = self.gripper_offset
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = factor
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

        # Generate actual controls to determine the orientation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        return int_marker


if __name__ == '__main__':
    rospy.init_node("SSBGraspMarker", log_level=rospy.INFO)
    # sm = SSBGraspedMarker()
    # gm = SSBGraspMarker(sm)
    SSBGraspedMarker()
    rospy.spin()
