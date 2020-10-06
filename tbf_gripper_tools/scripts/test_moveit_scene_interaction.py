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

from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle


def wait_till_updated(pl_scene, obj_name, attached, known):
    start = rospy.get_time()
    seconds = rospy.get_time()
    timeout = 5
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = pl_scene.get_attached_objects([obj_name])
        is_attached = len(attached_objects.keys()) > 0
        rospy.logdebug(
            "[MoveitInterface.wait_till_updated()] Attached Objects: %s" % attached_objects.keys())

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = obj_name in pl_scene.get_known_object_names()
        rospy.logdebug(
            "[MoveitInterface.wait_till_updated()] Known Objects: %s" % pl_scene.get_known_object_names())

        # Test if we are in the expected state
        if (attached == is_attached) and (known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.logdebug("[MoveitInterface.wait_till_updated()] Waiting ...")
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


def import_mesh(filename, scale=(1, 1, 1)):
    import pyassimp
    scene = pyassimp.load(filename)
    mesh = Mesh()
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, '__len__'):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, 'indices'):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face.indices[0],
                                           face.indices[1],
                                           face.indices[2]]
                mesh.triangles.append(triangle)
    else:
        return None
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0] * scale[0]
        point.y = vertex[1] * scale[1]
        point.z = vertex[2] * scale[2]
        mesh.vertices.append(point)
    # pyassimp.release(mesh)
    return mesh


if __name__ == '__main__':
    rospy.init_node("test_moveit_scene_interaction", log_level=rospy.DEBUG)

    service_name = 'apply_planning_scene'
    rospy.wait_for_service(service_name)
    apply_planning_scene = rospy.ServiceProxy(service_name, ApplyPlanningScene)

    add_request = ApplyPlanningSceneRequest()
    remove_request = ApplyPlanningSceneRequest()

    size = [0.75, 0.5, 0.5]
    p = Pose()
    p.position.x = 0
    p.position.y = size[1] / 2.0
    p.position.z = size[2] / 2.0

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size

    co = CollisionObject()
    co.header.frame_id = "controlbox_structure_top_front_link"
    co.id = "rubber"
    co.primitives = [box]
    co.primitive_poses = [p]
    co.operation = CollisionObject.ADD

    aco = AttachedCollisionObject()
    aco.link_name = "gripper_ur5_base_link"  # co.header.frame_id
    aco.object = co

    add_request.scene.is_diff = True
    add_request.scene.world.collision_objects = [co]
    add_request.scene.robot_state.attached_collision_objects = [aco]
    add_request.scene.robot_state.is_diff = True

    rco = CollisionObject()
    rco.id = co.id
    rco.operation = CollisionObject.REMOVE

    raco = AttachedCollisionObject()
    raco.link_name = aco.link_name
    raco.object = rco

    remove_request.scene.is_diff = True
    remove_request.scene.world.collision_objects = [rco]
    remove_request.scene.robot_state.attached_collision_objects = [raco]
    remove_request.scene.robot_state.is_diff = True

    while not rospy.is_shutdown():
        rospy.loginfo("[AddRequest] %s" % add_request)
        apply_planning_scene(add_request)
        rospy.sleep(2.0)
        rospy.loginfo("[RemoveRequest] %s" % remove_request)
        apply_planning_scene(remove_request)
        rospy.sleep(2.0)
    ####
    # mvit.clear_octomap_via_box_marker()

    # lst_equipment = SmartEquipment.from_parameter_server(group_name="~smart_equipment")
    # for eq in lst_equipment:
    #     mvit.add_equipment(eq)
    # while not rospy.is_shutdown():
    #     mvit._set_start_state()
    #     for selected_equipment in lst_equipment:
    #         mvit.attach_equipment(selected_equipment)
    #         rospy.sleep(2.0)
    #         mvit.detach_equipment()
    #         rospy.sleep(2.0)
    # rospy.spin()
