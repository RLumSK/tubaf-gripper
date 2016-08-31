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
import sys
import threading


def q_to_array(q):
    "ros Quaternion to array"
    return [q.x, q.y, q.z, q.w]

def p_to_array(p):
    "ros Point (or Vector3) to array"
    return [p.x, p.y, p.z]


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


def get_best_representative(Y,T,c):
    sq_Y = dist.squareform(Y,force='tomatrix',checks=False)
    clusters = np.max(T)
    res = np.array([sys.float_info.max] * clusters)
    n = len(T)
    assert n == sq_Y.shape[0]
    assert c -1 < clusters

    v_zero = np.zeros(n)
    # set elements not in the desired cluster to zero
    for i in range(n):
        if T[i] != c:
            sq_Y[i,:] = v_zero
            sq_Y[:,i] = v_zero

    min_idx = 0
    min_val = sys.float_info.max

    # square all distance values
    sq_Y = sq_Y**2

    for i in range(n):
        if T[i] == c:
            sum_sq = np.sum(sq_Y[i,:])
            if sum_sq < min_val:
                min_val = sum_sq
                min_idx = i

    return min_idx, min_val / float(n)


class PoseSamples:
    def __init__(self):
        # list of samples, elements are vectors of length 4 (quaternion, o) or 3 (translation)
        self.o = list()
        self.t = list()
        # the current best representative from last optimization
        self.best_o = None
        self.best_t = None
        # number of samples in list after last optimization (skip opt if no samples were added)
        self.last_num_o = 0
        self.last_num_t = 0
        pass

    def __str__(self):
        return "o: %d, t: %d" % (len(self.o), len(self.t))

    def __repr__(self):
        return self.__str__()

def print_current_transforms():
    for id_tup,pose_sample in iter(g.transforms.items()):
        rospy.loginfo("%-7s r: %52s  t: %s",str(id_tup), pose_sample.best_o,pose_sample.best_t)

# all configuration goes here
class cfg:
    min_samples = 12
    max_samples = 60
    cluster_threshold = 0.0005


# singleton class for global variables
class g:
    marker_sub = None
    transforms = defaultdict(lambda: PoseSamples())
    # otimization and adding of data happens asynchronous, so locking is required to not mess up the data
    transforms_lock = threading.Lock()


def on_markers(markers):
    mks = sorted(markers.markers,key=lambda m:m.id)
    m_len = len(mks)
    if m_len < 2:
        # rospy.logdebug("need at least 2 markers per view, view skipped")
        return

    # lock access to g.transforms
    g.transforms_lock.acquire()
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
            pose_sample.o.append(q_to_array(p01.orientation))
            pose_sample.t.append(p01.position)
    g.transforms_lock.release()

def optimize_clustering_orientations(id_tuple, do_print=True):
    pose_sample = g.transforms[id_tuple]
    data = np.array(pose_sample.o)
    n_samples = data.shape[0]
    if n_samples < cfg.min_samples:
        rospy.loginfo("not enough samples for pair %s",str(id_tuple))
        return
    if pose_sample.last_num_o == n_samples:
        rospy.loginfo("no new samples for %s", str(id_tuple))
        return

    # compute pdist (Y)
    Y = dist.pdist(data,metric=phi4_q)
    # linkage clustering (Z)
    Z = ch.linkage(Y,method='complete')
    # flatten these clusters based on distance with cfg.cluster_threshold
    T = ch.fcluster(Z,cfg.cluster_threshold,criterion='distance')
    # from these clusters, select the one with most members
    counts = np.bincount(T)
    largest = np.argmax(counts)
    # print "clusters: ", counts
    # compute best element from the cluster
    #   sum-square distances to other elements in cluster and take the one with lowest value
    best_idx,mean_square_dist = get_best_representative(Y,T,largest)
    best_repr = data[best_idx,:]
    g.transforms[id_tuple].best_o = best_repr
    #    first convert data to a list. -> removal of elements more efficient
    #    also write back data as list for efficient adding of samples
    data = list(data)

    # only remove samples, when over max_samples -- so it is possible to recover from a wrong initialisation
    if len(data) > cfg.max_samples:
        # remove all samples from other clusters
        #    iterate T i descending order, otherwise removing elements would mess up indices
        for i in range(len(T) -1,-1,-1):
            if T[i] != largest:
                data.pop(i)
        # optionally remove samples furthest away from best element when beyond max_samples
        remove_n = len(data) - cfg.max_samples
        # print best_repr
        # print remove_n
        if remove_n > 0:
            # remove samples furthest away from best representative.
            # data has probably changed, so just compute 1 to n distance again
            D = [phi4_q(best_repr,x) for x in data]
            indices_of_largest = np.argsort(D)[-remove_n:]
            # print "d", D
            # print "l", indices_of_largest
            for idx in np.sort(indices_of_largest)[::-1]:
                data.pop(idx)

    # finally write back (possibly pruned) data set
    g.transforms[id_tuple].o = data
    g.transforms[id_tuple].last_num_o = len(data)

    #and print some results
    if do_print:
        rospy.loginfo("optimization result for orientations:\n"
                      "  started with %d samples of %s\n"
                      "  found %d clusters, selected largest cluster with %d elements\n"
                      "  selected best representative %s with mean square distance of %f\n"
                      "  sample set size after optimization %d",
                      n_samples, str(id_tuple), len(counts),counts[largest],best_repr,mean_square_dist,len(data))



def optimize_clustering_orientations(id_tuple, do_print=True):
    pose_sample = g.transforms[id_tuple]
    data = np.array(pose_sample.o)
    n_samples = data.shape[0]
    if n_samples < cfg.min_samples:
        rospy.loginfo("not enough samples for pair %s",str(id_tuple))
        return
    if pose_sample.last_num_o == n_samples:
        rospy.loginfo("no new samples for %s", str(id_tuple))
        return

    # compute pdist (Y)
    Y = dist.pdist(data,metric=phi4_q)
    # linkage clustering (Z)
    Z = ch.linkage(Y,method='complete')
    # flatten these clusters based on distance with cfg.cluster_threshold
    T = ch.fcluster(Z,cfg.cluster_threshold,criterion='distance')
    # from these clusters, select the one with most members
    counts = np.bincount(T)
    largest = np.argmax(counts)
    # print "clusters: ", counts
    # compute best element from the cluster
    #   sum-square distances to other elements in cluster and take the one with lowest value
    best_idx,mean_square_dist = get_best_representative(Y,T,largest)
    best_repr = data[best_idx,:]
    g.transforms[id_tuple].best_o = best_repr
    #    first convert data to a list. -> removal of elements more efficient
    #    also write back data as list for efficient adding of samples
    data = list(data)

    # only remove samples, when over max_samples -- so it is possible to recover from a wrong initialisation
    if len(data) > cfg.max_samples:
        # remove all samples from other clusters
        #    iterate T i descending order, otherwise removing elements would mess up indices
        for i in range(len(T) -1,-1,-1):
            if T[i] != largest:
                data.pop(i)
        # optionally remove samples furthest away from best element when beyond max_samples
        remove_n = len(data) - cfg.max_samples
        # print best_repr
        # print remove_n
        if remove_n > 0:
            # remove samples furthest away from best representative.
            # data has probably changed, so just compute 1 to n distance again
            D = [phi4_q(best_repr,x) for x in data]
            indices_of_largest = np.argsort(D)[-remove_n:]
            # print "d", D
            # print "l", indices_of_largest
            for idx in np.sort(indices_of_largest)[::-1]:
                data.pop(idx)

    # finally write back (possibly pruned) data set
    g.transforms[id_tuple].o = data
    g.transforms[id_tuple].last_num_o = len(data)

    #and print some results
    if do_print:
        rospy.loginfo("optimization result for orientations:\n"
                      "  started with %d samples of %s\n"
                      "  found %d clusters, selected largest cluster with %d elements\n"
                      "  selected best representative %s with mean square distance of %f\n"
                      "  sample set size after optimization %d",
                      n_samples, str(id_tuple), len(counts),counts[largest],best_repr,mean_square_dist,len(data))

def on_optimize(timer_event):
    rospy.loginfo("running pose cluster optimization")

    g.transforms_lock.acquire()
    for k,v in iter(g.transforms.items()):
        # rospy.loginfo("   %s --- %s",str(k),str(v))
        optimize_clustering_orientations(k)

    print_current_transforms()
    g.transforms_lock.release()

if __name__ == '__main__':
    rospy.init_node("marker_reg",log_level=rospy.DEBUG)

    g.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_markers, queue_size=3)
    g.optim_timer = rospy.Timer(rospy.Duration(3), on_optimize)

    rospy.logdebug("Debug is enabled")
    rospy.spin()
    pass
