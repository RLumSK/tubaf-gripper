#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from collections import deque, defaultdict
import tf.transformations as tft
import scipy.cluster.hierarchy as ch
import scipy.spatial.distance as dist


def q_to_a(q):
    return [q.x, q.y, q.z, q.w]


# distance metrics based on "Metrics for 3D Rotations: Comparison and Analysis"
def phi3_q(q1, q2):
    # arccos( |q1•q2| )
    return np.arccos(np.abs(np.dot(q1, q2)))


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


def affine_to_pose(a):
    t = Point(*a[0:3, 3])
    o = Quaternion(*tft.quaternion_from_matrix(a))
    return Pose(position=t, orientation=o)


def pose_rel(p0, p1):
    """
    assume both poses in the same coordinate frame,
    compute a new pose transform that represents p0->p1 in p0's coordinate frame.
    basically inverse(p0) * p1
    """
    a0 = pose_to_affine(p0)
    a1 = pose_to_affine(p1)
    res = np.dot(tft.inverse_matrix(a0), a1)
    return affine_to_pose(res)


class PoseSamples:
    def __init__(self):
        self.o = list()
        self.t = list()
        self.best_o = None
        self.best_t = None
        pass

    def __str__(self):
        return "o: %d, t: %d" % (len(self.o), len(self.t))

    def __repr__(self):
        return self.__str__()


# all configuration goes here
class cfg:
    min_samples = 12
    max_samples = 120
    cluster_threshold = 0.00001


# singleton class for global variables
class g:
    marker_sub = None
    transforms = defaultdict(lambda: PoseSamples())


def on_markers(markers):
    mks = sorted(markers.markers,key=lambda m:m.id)
    m_len = len(mks)
    if m_len < 2:
        rospy.logdebug("need at least 2 markers per view, view skipped")
        return

    # compute relative poses
    for i in range(0, m_len - 1):
        mk0 = mks[i]
        p0 = mk0.pose.pose
        for j in range(i + 1, m_len):
            mk1 = mks[j]
            p1 = mk1.pose.pose
            p01 = pose_rel(p0, p1)
            key = (mk0.id, mk1.id)
            rospy.logdebug("adding pose sample for %d->%d", *key)
            pose_sample = g.transforms[key]
            pose_sample.o.append(p01.orientation)
            pose_sample.t.append(p01.position)

    pass

def cluster_orientations(rot_q_np):
    rospy.loginfo("clustering orientations from %s values", str(rot_q_np.shape))

    print "data:\n", rot_q_np

    print "shape ", rot_q_np.shape
    # noinspection PyTypeChecker


    clusters = ch.fclusterdata(rot_q_np, 1.8, metric=phi4_q, depth=4)
    print "clusters: ", clusters

    dm = dist.pdist(rot_q_np, metric=phi4_q)
    print "dm:", dm


    l = ch.linkage(dm,method='complete')
    print "linkage: ", l

    return clusters,dm,l

def optimize_clustering_orientations(id_tuple):
    data = np.array(g.transforms[id_tuple].o)

    # compute pdist (Y)
    Y = dist.pdist(data,metric=phi4_q)
    # linkage clustering (Z)
    Z = ch.linkage(Y,method='complete')
    # select biggest cluster up to cfg.cluster_threshold

    # flatten these clusters
    C = ch.fcluster(Z,cfg.cluster_threshold,criterion='distance')
    # from these clusters, select the one with most members
    # compute best element from the cluster
    #   sum-square distances to other elements in cluster and take the one with lowest value
    # remove all samples from other clusters
    # optionally remove samples furthest away from best element when beyond max_samples





def on_optimize(timer_event):
    rospy.loginfo("running pose cluster optimization")
    for k,v in iter(g.transforms.items()):
        rospy.loginfo("   %s --- %s",str(k),str(v))
        o_as_numpy = np.array(map(q_to_a,v.o))
        cluster_orientations(o_as_numpy)


if __name__ == '__main__':
    rospy.init_node("marker_reg",log_level=rospy.DEBUG)

    g.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_markers, queue_size=3)
    g.optim_timer = rospy.Timer(rospy.Duration(3), on_optimize)

    rospy.logdebug("Debug is enabled")
    rospy.spin()
    pass
