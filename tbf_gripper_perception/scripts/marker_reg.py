#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from collections import deque
import tf.transformations as tft

# distance metrics based on "Metrics for 3D Rotations: Comparison and Analysis"
def sigma3_q(q1,q2):
    #todo
    pass



def pose_to_affine(pose):
    pp = pose.position
    po = pose.orientation
    r = tft.quaternion_matrix([po.x, po.y, po.z, po.w])
    # tft already represents this matrix as 4x4, so just add the translation and be done
    r[:, 3] = [pp.x, pp.y, pp.z, 1.0]
    return r


def affine_to_pose(a):
    t = Point(*a[0:3,3])
    o = Quaternion(*tft.quaternion_from_matrix(a))
    return Pose(position=t,orientation=o)



def pose_rel(p0, p1):
    """
    assume both poses in the same coordinate frame,
    compute a new pose transform that represents p0->p1 in p0's coordinate frame.
    basically inverse(p0) * p1
    """
    a0 = pose_to_affine(p0)
    a1 = pose_to_affine(p1)


# all configuration goes here
class cfg:
    num_samples = 20


# singleton class for global variables
class g:
    marker_sub = None
    transforms = dict()


def on_markers(markers):
    m = markers.markers
    if len(m) < 2:
        rospy.logdebug("need at least 2 markers per view, view skipped")
        return

    # compute relative poses
    for mk in m:
        print pose_to_affine(mk.pose.pose)

    pass


def on_optimize():
    pass


if __name__ == '__main__':
    rospy.init_node("marker_reg")

    g.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_markers, queue_size=3)
    g.optim_timer = rospy.Timer(rospy.Time(2), on_optimize)
    rospy.spin()
    pass
