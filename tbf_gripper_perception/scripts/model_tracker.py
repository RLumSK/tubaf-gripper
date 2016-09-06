#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from urdf_parser_py.urdf import URDF
import tf
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import re
from ar_track_alvar_msgs.msg import AlvarMarkers

MARKER_NAME_PATTERN = re.compile("^marker\d+$")
ZERO_TIME = rospy.Time(0)


class g:
    tf_model = tf.TransformerROS()
    base_link = ""
    marker_links = set()


def mk_tft(src, dst, xyz, rpy, ):
    t = TransformStamped()
    t.header.frame_id = src
    t.child_frame_id = dst
    t.transform.rotation = Quaternion(*tft.quaternion_from_euler(*rpy))
    t.transform.translation = Vector3(*xyz)
    return t


def process_model(urdf_model, model_name):
    # get base link name from robot name when nothing is explicitly passed
    if model_name is None:
        model_name = urdf_model.name

    g.base_link = "%s_base_link" % model_name
    rospy.loginfo("using [%s] as model reference link. Pose of this link will be returned.", g.base_link)

    # add all the joints to our internal tf model, remember any link, that is named marker<id>
    for jt in urdf_model.joints:
        if jt.type != 'fixed':
            rospy.logwarn("only fixed joints supported, ignoring joint! %s -> %s is of type %s",
                          jt.parent, jt.child, jt.type)
            continue

        if MARKER_NAME_PATTERN.match(jt.child):
            g.marker_links.add(jt.child)

        g.tf_model.setTransform(mk_tft(jt.parent, jt.child, jt.origin.xyz, jt.origin.rpy))

    # finally check if all found marker links are reachable from the model base link
    err = False
    for marker in g.marker_links:
        try:
            g.tf_model.lookupTransform(g.base_link, marker, ZERO_TIME)
        except tf.Exception as e:
            rospy.logerr("Error while checking connectivity to each marker! @marker %s: Exception: %s",marker,e)
            err = True

    if err:
        exit(-1)

    rospy.loginfo("finished processing urdf model. Found %d markers.",len(g.marker_links))

def on_markers(markers):
    mks = sorted(markers.markers, key=lambda m: m.id)
    m_len = len(mks)

if __name__ == '__main__':
    rospy.init_node("model_tracker")

    model_description = rospy.get_param("~model_description", None)
    base_name = rospy.get_param("~base_name", None)

    if model_description is None:
        rospy.logfatal("empty model_description. Please load urdf model of object to track as ~model_description!")
        exit(-2)

    model = URDF.from_xml_string(model_description)
    process_model(model, base_name)

    g.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_markers, queue_size=3)
