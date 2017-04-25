#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, TU Bergakademie Freiberg
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
from shape_msgs.msg import Mesh


class MeshImporter(object):
    """
    Class is used in combination with the mesh_importer node (C++) in tbf_gripper_tools. It aims to import a mesh and
    object name, to be further used in the perception and grasping pipeline
    """

    def __init__(self, mesh_name):
        """
        Default Constructor
        :param mesh_name: name of the mesh and the corresponding topic
        :type mesh_name: String
        """
        self.mesh = None
        self.name = mesh_name
        self.sub = rospy.Subscriber(mesh_name, Mesh, self._onNewMeshMsg)

    def _onNewMeshMsg(self, msg):
        """
        Callback for the mesh_message Subscriber
        :param msg: Message with the mesh of the object
        :type msg: shape_msgs.msg.Mesh
        :return: None
        """
        rospy.logdebug("[MeshImporter._onNewMeshMsg()] received msg for mesh_name: %s", self.name)
        self.mesh = msg

    def get_name(self):
        """
        :return: name of the imported mesh
        """
        return self.name

    def get_mesh(self):
        """
        :return: mesh of the object
        """
        return self.mesh

if __name__ == '__main__':
    rospy.init_node("mesh_importer_python")
    if rospy.has_param("~mesh_name"):
        mesh_name = rospy.get_param("~mesh_name")
        if type(mesh_name) is str:
            obj = MeshImporter(mesh_name)
            rospy.spin()
        elif type(mesh_name) is list:
            if type(mesh_name[0]) is str:
                objs = []
                for name in mesh_name:
                    objs.append(MeshImporter(name))
                rospy.spin()
            else:
                rospy.logwarn("[MeshImporter.main] 'mesh_name' is a list with type: %s elements",
                              type(mesh_name[0]))
        else:
            rospy.logwarn("[MeshImporter.main] 'mesh_name' is of type: %s ",
                          type(mesh_name[0]))
    else:
        rospy.logwarn("[MeshImporter.main] No 'mesh_name' set as parameter")
