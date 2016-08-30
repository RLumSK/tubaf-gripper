#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from collections import deque
import tf.transformations as tft
import marker_reg as mr
import scipy
from matplotlib import pyplot as plt
import scipy.cluster.hierarchy as ch
import scipy.spatial.distance as dist

rospy.init_node("fooooooo", anonymous=True, log_level=rospy.DEBUG)


def cluster_orientations(as_np):
    rospy.loginfo("clustering orientations from %s values",str(as_np.shape))

    print "data:\n", as_np

    print "shape ", as_np.shape
    # noinspection PyTypeChecker


    clusters = ch.fclusterdata(as_np,1.8,metric=mr.phi4_q,depth=4)
    print "clusters: ", clusters

    dm = dist.pdist(as_np,metric=mr.phi4_q)
    print "dm:", dm


    l = ch.linkage(dm,method='complete')
    print "linkage: ", l

    return clusters,dm,l

# t0 = np.array([1, 4, 0])
# t1 = np.array([3, 4, 0])
#
# r0 = tft.quaternion_from_euler(0, 0, -0.5 * np.pi)
# r1 = tft.quaternion_from_euler(0, 0, 0)
#
# print t0
# print r0
#
# print
#
# print t1
# print r1
#
# print
#
# t0h = np.hstack((t0, [1]))
#
# a0 = tft.quaternion_matrix(r0)
# a0[:, 3] = t0h
#
# t1h = np.hstack((t1, [1]))
# a1 = tft.quaternion_matrix(r1)
# a1[:, 3] = t1h
#
# print "pose 0"
# print a0
#
# print
# print "pose 1"
# print a1
#
# a01 = np.dot(tft.inverse_matrix(a0), a1)
#
# print
# print "pose1 relative to pose0 (p0->p1)"
# print a01
#
# print "-----"
#
# r0i = tft.quaternion_inverse(r0)
#
# print mr.affine_to_pose(a0)
#
# print mr.phi4_q(r0,r1)
# print mr.phi4_q(r1,r0)
# print mr.phi4_q(r0,r0)
#
# q0 = Quaternion(*r0)
# q1 = Quaternion(*r1)

data3 = np.array(
    [[-0.51653967, -0.51155577, -0.46026764, 0.50955976],
     [0.54098946, 0.49749909, 0.45866229, -0.49945366],
     [-0.53514326, -0.49844866, -0.45820428, 0.5051925],
     [0.52808099, 0.50342269, 0.46713339, -0.4994822],
     [0.52753396, 0.49707972, 0.48398966, -0.49027918],
     [0.48603178, 0.53996017, 0.49197517, -0.47976719],
     [0.49865856, 0.51847146, 0.5118329, -0.46963184],
     [-0.49861812, -0.51897013, -0.47703056, 0.50447182], ])
data = np.array(
    [[-4.85129346e-03, 7.37820966e-01, -5.54523932e-03, 6.74956249e-01],
     [1.27651204e-02, 7.34438367e-01, 1.41539286e-02, 6.78407697e-01],
     [8.01015336e-03, 7.33140784e-01, 1.76732352e-02, 6.79800033e-01],
     [5.47489668e-03, 7.35190020e-01, 3.99337865e-03, 6.77827200e-01],
     [3.46669700e-03, 7.34153919e-01, 5.96128623e-03, 6.78948060e-01],
     [1.60625827e-03, 7.34679828e-01, 7.05324266e-03, 6.78375428e-01],
     [2.47436392e-03, 7.33975257e-01, 3.38203653e-03, 6.79163280e-01],
     [-2.21678482e-03, 7.34936615e-01, 9.98195828e-03, 6.78058713e-01],
     [-4.30146068e-03, 7.37277632e-01, 9.87933744e-03, 6.75503952e-01],
     [-3.64483303e-04, 7.34624377e-01, 8.48843367e-03, 6.78420842e-01],
     [1.38043966e-03, 7.31728549e-01, 5.81921064e-03, 6.81569924e-01],
     [-2.28428816e-03, 7.32685908e-01, -4.32499005e-03, 6.80549364e-01],
     [9.99872189e-03, 7.31831673e-01, 1.11952792e-02, 6.81320111e-01],
     [2.90156460e-03, 7.33369314e-01, 4.42875532e-03, 6.79809838e-01],
     [1.08400247e-02, 7.26361709e-01, 1.32778544e-02, 6.87098873e-01],
     [8.12295850e-03, 7.29827676e-01, 1.28451612e-03, 6.83581693e-01],
     [1.09882801e-02, 7.28172733e-01, 1.67648170e-02, 6.85100481e-01],
     [1.52056938e-02, 7.32724634e-01, 1.48990364e-02, 6.80192191e-01],
     [2.01947208e-02, 7.31863920e-01, 1.18420601e-02, 6.81048560e-01],
     [7.85215995e-03, 7.33053464e-01, -3.95122823e-04, 6.80125582e-01],
     [1.48331660e-02, 7.33903536e-01, 9.53320761e-03, 6.79024812e-01],
     [1.20063139e-03, 7.34794237e-01, 8.45010510e-03, 6.78236377e-01],
     [-4.90185617e-03, 7.36582905e-01, -4.38462684e-03, 6.76315289e-01], ]
)

data2 = np.array(
    [[2.31445821e-02, 7.22095450e-01, 1.72824572e-02, 6.91190138e-01],
     [2.56969495e-02, 7.26310893e-01, 3.12132421e-02, 6.86176280e-01],
     [3.30841628e-02, 7.19458947e-01, 2.48654403e-02, 6.93300780e-01],
     [3.65808366e-02, 7.19805762e-01, 1.79572350e-02, 6.92978387e-01],
     [2.66709477e-02, 7.20576193e-01, 3.47933054e-03, 6.92853884e-01],
     [3.32310706e-02, 7.25673170e-01, 1.60817914e-02, 6.87048413e-01],
     [1.47535011e-02, 7.25537586e-01, 1.10374090e-02, 6.87935841e-01],
     [9.51200158e-03, 7.22346715e-01, 5.05711266e-04, 6.91465465e-01], ])

# data = np.vstack((data, data3))

q90 = tft.quaternion_from_euler(0, 0, np.pi * 0.5)

data5 = map(lambda q: tft.quaternion_multiply(q, q90), data2)

# data = np.vstack((data2, data5))

v = np.array([1, 0, 0, 1])

for row in data:
    m = tft.quaternion_matrix(row)
    print np.dot(m, v)

clu, dm, li = mr.cluster_orientations(data)

c, co = ch.cophenet(li, dm)
print "c ", c

print len(dm)

plt.figure(figsize=(25, 10))
plt.title('Hierarchical Clustering Dendrogram')
plt.xlabel('sample index')
plt.ylabel('distance')
ch.dendrogram(
    li,
    leaf_rotation=90.,  # rotates the x axis labels
    leaf_font_size=8,  # font size for the x axis labels
)
plt.show()

# find min and max distance in dist matrix
# take min dist, add a threshold factor
# find the pair with the min dist and add any readings to that with ascending dist
# form other clusters with elements with also very low dist, which cant be linked to the first pair
# finally take the largest cluster and use that

npl = data.tolist()


