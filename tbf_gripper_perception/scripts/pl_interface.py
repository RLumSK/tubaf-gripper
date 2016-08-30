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
import moveit_commander

import ar_track_alvar_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv

class PlaningInterface(object):

    def __init__(self):
        """
        Default constructor that loads parameter from the parameter server, register callbacks and publisher
        """
        rospy.loginfo("pl_interface.py:PlanningInterface() initializing")
        self.ar_topic = rospy.get_param("~ar_topic", "/ar_pose_marker")
        self.ar_sub = rospy.Subscriber( self.ar_topic, ar_track_alvar_msgs.msg.AlvarMarkers, callback=self.onMarkersMessage,
                                       queue_size=1)

        self.scene = moveit_commander.PlanningSceneInterface()


        self.stl = rospy.get_param("~model_path", "package://tbf_gripper_perception/meshes/wlan_box.stl")
        self.collision_scale = rospy.get_param("~model_scale", 1e-03)

        # http://answers.ros.org/question/157716/obstacles-management-in-moveit/
        self._pubPlanningScene = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene, queue_size=10)
        rospy.wait_for_service('/get_planning_scene', 10.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', moveit_msgs.srv.GetPlanningScene)
        request = moveit_msgs.msg.PlanningSceneComponents(components=moveit_msgs.msg.PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = get_planning_scene(request)
        allowed_collisions_matrix = response.scene.allowed_collision_matrix

        # allow collision
        if self.ar_topic not in allowed_collisions_matrix.default_entry_names:
            rospy.loginfo("pl_interface.py:PlanningInterface() add object to allowed collision matrix")
            # add button to allowed collision matrix
            # allowed_collisions_matrix.entry_values
            allowed_collisions_matrix.default_entry_names += [self.ar_topic]
            allowed_collisions_matrix.default_entry_values += [True]

            planning_scene_diff = moveit_msgs.msg.PlanningScene(
                is_diff=True,
                allowed_collision_matrix=allowed_collisions_matrix)

            self._pubPlanningScene.publish(planning_scene_diff)
            rospy.sleep(1.0)
        rospy.loginfo("pl_interface.py:PlanningInterface() initialized")

    def onMarkersMessage(self, msg):
        """
        After receiving a set of markers publish the referring collision object in the planning scene
        :param msg: message from the ar_track_alvar package
        :type msg: ar_track_alvar_msgs.msg.AlvarMarkers
        :return: -
        :rtype: -
        """
        # rospy.loginfo("pl_interface.py:PlanningInterface.onMarkersMessage() received msg:%s", msg)
        if len(msg.markers) < 1:
            return
        max_marker = msg.markers[0]
        # determine which pose gone serve as reference for the object
        # for i in range(1, len(msg.markers)):
        #     if msg.markers[i].confidence > confidence:
        #         max_marker = msg.markers[i]
        #         confidence = msg.markers[i].confidence
        # if max_marker is None:
        #     return

        # rospy.loginfo("pl_interface.py:PlanningInterface.onMarkersMessage() max marker:%s", max_marker)
        # add offset to ar_marker pose
        max_marker.pose.pose.position.z -= 0.078
        max_marker.pose.header = max_marker.header

        # add collision object at given pose
        cs = self.collision_scale*1.2
        self.scene.add_mesh(name=self.ar_topic, pose=max_marker.pose, filename=self.stl, size=(cs, cs, cs))
        rospy.loginfo("pl_interface.py:PlanningInterface.onMarkersMessage() finished")
        rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node("tbf_gripper_perception_planing_interface", anonymous=False)
    obj = PlaningInterface()

    rospy.spin()
