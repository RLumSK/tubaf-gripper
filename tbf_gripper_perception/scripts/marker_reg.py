#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, TransformStamped, Transform, Vector3
from std_msgs.msg import Header
from collections import deque, defaultdict
import tf.transformations as tft
import scipy.cluster.hierarchy as ch
import scipy.spatial.distance as dist
import sys
import threading
from tf import TransformerROS
import os
import pickle
from visualization_msgs.msg import Marker, MarkerArray
import copy
import scipy.sparse.csgraph as csg

def invert_affine(a):
    r = a[0:3,0:3]
    t = a[0:3,3]




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


    print np.hstack((a0,a1,res))

    return affine_to_pose(res)


def get_best_representative(Y, T, c):
    sq_Y = dist.squareform(Y, force='tomatrix', checks=False)
    clusters = np.max(T)
    n = len(T)
    assert n == sq_Y.shape[0]
    assert c - 1 < clusters

    v_zero = np.zeros(n)
    # set elements not in the desired cluster to zero
    for i in range(n):
        if T[i] != c:
            sq_Y[i, :] = v_zero
            sq_Y[:, i] = v_zero

    min_idx = 0
    min_val = sys.float_info.max

    # square all distance values
    sq_Y = sq_Y ** 2

    for i in range(n):
        if T[i] == c:
            sum_sq = np.sum(sq_Y[i, :])
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
        self.last_num = 0
        pass

    def __str__(self):
        return "o: %d, t: %d" % (len(self.o), len(self.t))

    def __repr__(self):
        return self.__str__()

    def best_transform(self):
        if self.best_o is None or self.best_t is None:
            return None
        m = tft.quaternion_matrix(self.best_o)
        m[0:3, 3] = self.best_t
        return m


def print_current_transforms():
    for id_tup, pose_sample in iter(g.transforms.items()):
        rospy.loginfo("%-9s r: %52s  t: %s", str(id_tup), pose_sample.best_o, pose_sample.best_t)
        # print pose_sample.best_transform()


def mk_tft(src, dst, orient_or_affine, trans=None, header=None):
    t = TransformStamped()
    if header is not None:
        t.header = header

    t.header.frame_id = src
    t.child_frame_id = dst

    if trans is not None:
        t.transform.rotation = Quaternion(*orient_or_affine)
        t.transform.translation = Vector3(*trans)
    else:
        p = affine_to_pose(orient_or_affine)
        t.transform.translation = p.position
        t.transform.rotation = p.orientation
    return t


def vis_result(src_transform, base_id):
    # build a transformer for easier getting marker poses
    h = Header()
    h.stamp = rospy.get_rostime()
    h.frame_id = "base_link"
    transformer = TransformerROS()
    transformer.setTransform(mk_tft("base_link", "id_%d" % base_id, src_transform))

    arr = MarkerArray()
    arr.markers = list()

    # draw all the markers
    m = copy.deepcopy(cfg.default_marker)
    m.header = h
    m.id = base_id
    pos, rot = transformer.lookupTransform("base_link","id_%d" % base_id, rospy.Time(0))
    m.pose.position = Point(*pos)
    m.pose.orientation = Quaternion(*rot)
    m.text = "id_%d" % base_id


    arr.markers.append(m)
    valid_ids = set()

    valid_ids.add(base_id)

    # n = np.max([ids[1] for ids in g.transforms.keys()]) +1
    # graph_mat = np.zeros((n,n))
    # for ids, pose_data in iter(g.transforms.items()):
    #     if pose_data.best_o is not None:
    #         graph_mat[ids[0],ids[1]] = graph_mat[ids[1],ids[0]] = 1.0 / float(pose_data.last_num)
    #
    # graph = csg.csgraph_from_dense(graph_mat)
    # print graph
    #
    # print "---"
    # spanningtree = csg.breadth_first_tree(graph,base_id,False)
    #
    # print spanningtree
    # exit(0)

    iters = 0
    remaining = set(g.transforms.keys())
    held_back = set()
    while iters < 2:
        for id_tup in remaining:
            pose_samples = g.transforms[id_tup]
            if pose_samples.best_o is None:
                continue

            if (id_tup[0] in valid_ids and id_tup[1] not in valid_ids):
                transformer.setTransform(mk_tft("id_%d" % id_tup[0], "id_%d" % id_tup[1], pose_samples.best_transform()))
                rospy.logdebug("added transform for %s", str(id_tup))
                iters = 0
                valid_ids.add(id_tup[1])
            elif (id_tup[1] in valid_ids and id_tup[0] not in valid_ids):
                transformer.setTransform(mk_tft("id_%d" % id_tup[1], "id_%d" % id_tup[0], tft.inverse_matrix(pose_samples.best_transform())))
                rospy.logdebug("added transform for swapped %s", str(id_tup))
                iters = 0
                valid_ids.add(id_tup[0])
            else:
                held_back.add(id_tup)
        remaining.update(held_back)
        held_back.clear()
        iters += 1


    # for ids, pose_data in iter(g.transforms.items()):
    #     if pose_data.best_o is not None:
    #         transformer.setTransform(mk_tft("id_%d" % ids[0], "id_%d" % ids[1], pose_data.best_transform()))
    #         rospy.logdebug("added transform for %s",str(ids))
    #         valid_ids.append(ids[1])

    for id in valid_ids:
        try:
            m = copy.deepcopy(cfg.default_marker)
            m.header = h
            m.id = id
            pos, rot = transformer.lookupTransform("base_link","id_%d" % id, rospy.Time(0))
            m.pose.position = Point(*pos)
            m.pose.orientation = Quaternion(*rot)
            arr.markers.append(m)
            m = copy.deepcopy(m)
            m.id = 1000+ m.id
            m.text = "id_%d" % id
            m.type = Marker.TEXT_VIEW_FACING
            arr.markers.append(m)
        except Exception as e:
            rospy.logwarn("vis error for %s! Reason: %s",str(id), e)

    # publish marker array
    g.reg_pub.publish(arr)


def save_data(fn):
    g.transforms_lock.acquire()
    try:
        pickle.dump(dict(g.transforms), open(fn, 'wb'))
        rospy.loginfo("saved transforms to %s", fn)
    except StandardError as e:
        rospy.logerr("save failed, %s", str(e))

    g.transforms_lock.release()


def load_data(fn):
    if not os.path.exists(fn):
        rospy.logerr("load failed, %s does not exist", fn)
        return
    try:
        tfms = pickle.load(open(fn, 'rb'))
    except StandardError as e:
        rospy.logerr("load failed, %s", str(e))
        return

    g.transforms_lock.acquire()
    if tfms is not None:
        rospy.loginfo("loaded transforms from %s", fn)
        g.transforms.clear()
        g.transforms.update(tfms)
    g.transforms_lock.release()


# all configuration goes here
class cfg:
    min_samples = 12
    max_samples = 60
    cluster_threshold = 0.0005

    default_marker = Marker()
    default_marker.ns = "object"
    default_marker.lifetime.secs = 10
    default_marker.color.a = 0.5
    default_marker.type = Marker.CUBE
    default_marker.scale = Vector3(0.1, 0.1, 0.01)


# singleton class for global variables
class g:
    marker_sub = None
    reg_pub = None
    transforms = defaultdict(lambda: PoseSamples())
    # optimization and adding of data happens asynchronous, so locking is required to not mess up the data
    # use reentrant lock for ease of use, calling subroutines from locked states which are also locking
    transforms_lock = threading.RLock()


def on_markers(markers):
    mks = sorted(markers.markers, key=lambda m: m.id)
    m_len = len(mks)
    if m_len < 2:
        # rospy.logdebug("need at least 2 markers per view, view skipped")
        return

    # lock access to g.transforms
    g.transforms_lock.acquire()
    # compute relative poses
    for i in range(0, m_len - 1):
        mk0 = mks[i]
        if mk0.id not in cfg.valid_ids:
            continue
        p0 = mk0.pose.pose
        for j in range(i + 1, m_len):
            mk1 = mks[j]
            if mk1.id not in cfg.valid_ids:
                continue
            p1 = mk1.pose.pose
            p01 = pose_rel(p0, p1)
            key = (mk0.id, mk1.id)
            rospy.logdebug("adding pose sample for %d->%d", *key)
            pose_sample = g.transforms[key]
            pose_sample.o.append(q_to_array(p01.orientation))
            pose_sample.t.append(p_to_array(p01.position))

    g.transforms_lock.release()


def optimize_clustering(id_tuple, sample_data, metric, distance_threshold, print_info=None):
    # pose_sample = g.transforms[id_tuple]
    data = np.array(sample_data)
    n_samples = data.shape[0]
    if n_samples < cfg.min_samples:
        rospy.loginfo("not enough samples for pair %s", str(id_tuple))
        return None

    # compute pdist (Y)
    Y = dist.pdist(data, metric=metric)
    # linkage clustering (Z)
    Z = ch.linkage(Y, method='complete')
    # flatten these clusters based on distance with cfg.cluster_threshold
    T = ch.fcluster(Z, distance_threshold, criterion='distance')
    # from these clusters, select the one with most members
    counts = np.bincount(T)
    largest = np.argmax(counts)
    # print "clusters: ", counts
    # compute best element from the cluster
    #   sum-square distances to other elements in cluster and take the one with lowest value
    best_idx, mean_square_dist = get_best_representative(Y, T, largest)
    best_repr = data[best_idx, :]
    #    first convert data to a list. -> removal of elements more efficient
    #    also write back data as list for efficient adding of samples
    data = list(data)

    # only remove samples, when over max_samples -- so it is possible to recover from a wrong initialisation
    if len(data) > cfg.max_samples:
        # remove all samples from other clusters
        #    iterate T i descending order, otherwise removing elements would mess up indices
        for i in range(len(T) - 1, -1, -1):
            if T[i] != largest:
                data.pop(i)
        # optionally remove samples furthest away from best element when beyond max_samples
        remove_n = len(data) - cfg.max_samples
        # print best_repr
        # print remove_n
        if remove_n > 0:
            # remove samples furthest away from best representative.
            # data has probably changed, so just compute 1 to n distance again
            D = [phi4_q(best_repr, x) for x in data]
            indices_of_largest = np.argsort(D)[-remove_n:]
            # print "d", D
            # print "l", indices_of_largest
            for idx in np.sort(indices_of_largest)[::-1]:
                data.pop(idx)

    # and print some results
    if print_info is not None:
        rospy.loginfo("optimization result for orientations:\n"
                      "  %s\n"
                      "  started with %d samples of %s\n"
                      "  found %d clusters, selected largest cluster with %d elements\n"
                      "  selected best representative %s with mean square distance of %f\n"
                      "  sample set size after optimization %d",
                      print_info, n_samples, str(id_tuple), len(counts), counts[largest],
                      best_repr, mean_square_dist, len(data))
    # return the best representative of largest cluster and the possibly pruned data
    return best_repr, data


def on_optimize(timer_event):
    rospy.loginfo("running pose cluster optimization")

    g.transforms_lock.acquire()
    for id_tuple, pose_samples in iter(g.transforms.items()):
        # rospy.loginfo("   %s --- %s",str(k),str(v))
        if len(pose_samples.o) == pose_samples.last_num:
            # skip due to no change in sample count
            rospy.loginfo("skip %9s, no new samples", str(id_tuple))
            continue

        res_o = optimize_clustering(id_tuple, pose_samples.o, phi4_q, cfg.cluster_threshold, "orientations")
        if res_o is not None:
            pose_samples.best_o, pose_samples.o = res_o
            pose_samples.last_num = len(pose_samples.best_o)

        res_t = optimize_clustering(id_tuple, pose_samples.t, 'euclidean', cfg.cluster_threshold, "translations")
        if res_t is not None:
            pose_samples.best_t, pose_samples.t = res_t

    if cfg.autosave:
        save_data(cfg.datafile)
    print_current_transforms()

    base_tf = np.eye(4)
    base_tf[1, 3] = -0.5
    vis_result(base_tf, 20)

    g.transforms_lock.release()


if __name__ == '__main__':
    rospy.init_node("marker_reg", log_level=rospy.DEBUG)

    cfg.datafile = rospy.get_param("~datafile","/tmp/marker_reg.pypickle")
    cfg.autosave = rospy.get_param("~autosave",True)
    cfg.sizes = rospy.get_param("~marker_sizes", dict())
    cfg.valid_ids = set([int(s[3:]) for s in cfg.sizes.keys()])

    # load_data("/tmp/marker_reg.pypickle")
    load_data(cfg.datafile)

    print cfg.valid_ids
    for tp in g.transforms.keys():
        if tp[0] not in cfg.valid_ids or tp[1] not in cfg.valid_ids:
            rospy.loginfo("removed invalid id %s",str(tp))
            g.transforms.pop(tp)

    # t0 = PoseSamples()
    # t1 = PoseSamples()
    # t2 = PoseSamples()
    # t0.best_o = [0.0, 0.0, 0.0, 1.0]
    # t0.best_t = [0.1, 0.0, 0.0]
    #
    # t1.best_o = tft.quaternion_from_euler(np.pi * 0.5,0,0)
    # t1.best_t = [-0.1, 0.0, 0.0]
    #
    # t2.best_o = tft.quaternion_from_euler(0, np.pi * 0.5, 0)
    # t2.best_t = [0.1, 0.1, 0.0]
    #
    # g.transforms[(20, 21)] = t0
    # g.transforms[(20, 22)] = t1
    # g.transforms[(20, 23)] = t2

    g.reg_pub = rospy.Publisher("/reg_marker_pub", MarkerArray, queue_size=1)


    g.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_markers, queue_size=3)
    g.optim_timer = rospy.Timer(rospy.Duration(3), on_optimize)

    rospy.logdebug("Debug is enabled")
    rospy.spin()
    pass
