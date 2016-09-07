#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from urdf_parser_py.urdf import URDF
import tf
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point, Pose, PoseStamped
import re
from ar_track_alvar_msgs.msg import AlvarMarkers
import threading
import numpy as np
from collections import deque
import sys

MARKER_NAME_PATTERN = re.compile("^marker\d+$")
ZERO_TIME = rospy.Time(0)
TF_PUB_DUR = rospy.Duration(0, 1e8)


def phi4_q(q1, q2):
    # 1 - |q1•q2|
    return 1 - np.abs(np.dot(q1, q2))


def pose_to_affine(pose):
    pp = pose.position
    po = pose.orientation
    r = tft.quaternion_matrix([po.x, po.y, po.z, po.w])
    # tft already represents this matrix as 4x4, so just add the translation and be done
    r[:, 3] = [pp.x, pp.y, pp.z, 1.0]
    return r


def q_to_array(q):
    "ros Quaternion to array"
    return [q.x, q.y, q.z, q.w]


def p_to_array(p):
    "ros Point (or Vector3) to array"
    return np.array([p.x, p.y, p.z])


def affine_to_pose(a):
    t = Point(*a[0:3, 3])
    o = Quaternion(*tft.quaternion_from_matrix(a))
    return Pose(position=t, orientation=o)


class g:
    tf_model = tf.TransformerROS()
    # model base link
    base_link = ""
    # sets of marker links and ids parsed from urdf
    marker_links = set()
    marker_ids = set()
    # keep track of when tf was last updated
    publish_tf = True
    tf_last_pub = rospy.Time(0)
    tf_cur_transform = TransformStamped()
    # protect tf publishing, called from timer and marker callback
    ft_transform_lock = threading.Lock()
    tf_broadcaster = None
    model_pose_pub = None
    tf_model_lookup_affine_inverse_cache = dict()
    tf_model_lookup_affine_cache = dict()
    last_pose = Pose(orientation=Quaternion(0, 0, 0, 1))


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
            rospy.logerr("Error while checking connectivity to each marker! @marker %s: Exception: %s", marker, e)
            err = True

    if err:
        exit(-1)

    rospy.loginfo("finished processing urdf model. Found %d markers.", len(g.marker_links))


def on_tf_pub_timer(timer_event):
    g.ft_transform_lock.acquire()

    cur_time = rospy.get_rostime()
    if cur_time - g.tf_last_pub > TF_PUB_DUR:
        t = g.tf_cur_transform
        t.header.stamp = cur_time
        t.header.seq += 1
        g.tf_broadcaster.sendTransformMessage(t)
        g.tf_last_pub = cur_time

    g.ft_transform_lock.release()


def publish_pose(pose, header):
    angle_dif = phi4_q(q_to_array(pose.orientation), q_to_array(g.last_pose.orientation))
    pos_dif = abs(np.linalg.norm(p_to_array(pose.position)) - np.linalg.norm(p_to_array(g.last_pose.position)))

    rospy.loginfo("pose dif r: %f    t: %f", angle_dif, pos_dif)

    ps = PoseStamped()
    ps.header = header
    ps.pose = pose
    if angle_dif + pos_dif < 0.001:
        g.model_pose_pub.publish(ps)

        if g.publish_tf:
            g.ft_transform_lock.acquire()
            t = g.tf_cur_transform
            t.transform.rotation = pose.orientation
            t.transform.translation = pose.position
            t.header = header
            g.tf_broadcaster.sendTransformMessage(t)
            g.tf_last_pub = rospy.get_rostime()
            g.ft_transform_lock.release()

    g.last_pose = pose


def lookup_tf_model_affine_inverse(id):
    """lookup transform from any marker (with given id) to the model base link."""
    A = g.tf_model_lookup_affine_inverse_cache.get(id, None)
    if A is not None:
        return A
    t, R = g.tf_model.lookupTransform("marker%d" % id, g.base_link, ZERO_TIME)
    A = tft.quaternion_matrix(R)
    A[0:3, 3] = t
    g.tf_model_lookup_affine_inverse_cache[id] = A
    return A


def lookup_tf_model_affine(id):
    A = g.tf_model_lookup_affine_cache.get(id, None)
    if A is not None:
        return A
    t, R = g.tf_model.lookupTransform(g.base_link, "marker%d" % id, ZERO_TIME)
    A = tft.quaternion_matrix(R)
    A[0:3, 3] = t
    g.tf_model_lookup_affine_cache[id] = A
    return A


def normalize(v):
    return v / np.linalg.norm(v)


def vector_rotation(v1, v2):
    """return R so v2 = R*v1"""
    v1 = normalize(v1)
    v2 = normalize(v2)
    axis = np.cross(v1, v2)
    cos_angle = np.dot(v1, v2)

    q = tft.quaternion_about_axis(np.arccos(cos_angle), axis)
    return q


def on_markers(markers):
    mks = {m.id: m for m in markers.markers}

    found_ids = set(mks.keys())
    found_known = g.marker_ids.intersection(found_ids)
    n_found = len(found_known)

    if n_found == 0:
        # no marker -> no alignment
        rospy.logdebug("no known markers detected (%d of %d)", n_found, len(found_ids))
    elif n_found == 1:
        # single marker -> align to that
        m = mks[found_known.pop()]  # single element, don't need it afterwards, so this is ok
        rospy.logdebug("single known marker detected. Align object to marker %d.", m.id)
        # get transform from marker to obj-base (in that direction)
        A_model = lookup_tf_model_affine_inverse(m.id)
        # print A
        A_marker = pose_to_affine(m.pose.pose)
        A_res = np.dot(A_marker, A_model)
        res_pose = affine_to_pose(A_res)

        publish_pose(res_pose, m.header)
        # elif n_found == 2:
        # two markers: compute average pose and derive orientation from positions
    else:
        # 3 or more markers, hmmm...    for now just take any two of them and compute pose of them
        # copy this to the case of two markers when implementation with more then two is ready
        m0 = mks[found_known.pop()]
        m1 = mks[found_known.pop()]
        rospy.logdebug("multiple known marker detected. Align object to markers %d and %d.", m0.id, m1.id)
        tc_c0 = lookup_tf_model_affine(m0.id)[0:3, 3]
        tc_c1 = lookup_tf_model_affine(m1.id)[0:3, 3]

        tc_01 = tc_c1 - tc_c0
        print tc_01

        tb_b0 = p_to_array(m0.pose.pose.position)
        tb_b1 = p_to_array(m1.pose.pose.position)

        tb_01 = tb_b1 - tb_b0
        print tb_01

        r = vector_rotation(tc_01, tb_01)
        R = tft.quaternion_matrix(r)
        print r

        h = np.dot(R, np.concatenate((tc_c0, [1])))
        h = h[0:3] / h[3]  # dehomogenize
        tb_bc_0 = tb_b0 - h

        h = np.dot(R, np.concatenate((tc_c1, [1])))
        h = h[0:3] / h[3]  # dehomogenize
        tb_bc_1 = tb_b1 - h

        tb_bc = (tb_bc_0 + tb_bc_1) / 2

        pose = Pose(position=Vector3(*tb_bc), orientation=Quaternion(*r))
        publish_pose(pose, m0.header)


def main():
    if "debug" in sys.argv:
        rospy.init_node("model_tracker", log_level=rospy.DEBUG)
    else:
        rospy.init_node("model_tracker")

    model_description = rospy.get_param("~model_description", None)
    base_name = rospy.get_param("~base_name", None)
    g.publish_tf = rospy.get_param("~publish_tf", True)

    default_tf = rospy.get_param("~default_transform", "base_link 0 0 0.5 0 0 0")

    if model_description is None:
        rospy.logfatal("empty model_description. Please load urdf model of object to track as ~model_description!")
        exit(-2)

    model = URDF.from_xml_string(model_description)
    process_model(model, base_name)

    # just create an int-set of ids from known links for easier and faster lookup
    g.marker_ids = set([int(m[6:]) for m in g.marker_links])

    # after parsing the model, we also know the model base link...
    # init default transform, setup tf pub timer (if required)
    if g.publish_tf:
        dtf_parts = default_tf.split()
        if len(dtf_parts) != 7:
            rospy.logfatal("default transform has wrong format. Must be [src_frame x y z r p y]. "
                           "Similar to static transform publisher, but just 7 values.\n"
                           "But got: %s", default_tf)
            exit(-1)
        g.tf_cur_transform = mk_tft(dtf_parts[0], g.base_link, map(float, dtf_parts[1:4]), map(float, dtf_parts[4:7]))
        g.tf_broadcaster = tf.TransformBroadcaster(queue_size=10)
        g.tf_pub_timer = rospy.Timer(rospy.Duration(0, 1e8), on_tf_pub_timer)

    g.model_pose_pub = rospy.Publisher(rospy.get_name() + "/ar_model_pose", PoseStamped, queue_size=3)
    # finally subscribe to marker poses
    g.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_markers, queue_size=3)
    rospy.spin()


if __name__ == '__main__':
    # use separate function to not make stuff global by accident
    main()
