#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from collections import deque
import tf.transformations as tft
import marker_reg as mr

t0 = np.array([1, 4, 0])
t1 = np.array([3, 4, 0])

r0 = tft.quaternion_from_euler(0, 0, -0.5 * np.pi)
r1 = tft.quaternion_from_euler(0, 0, 0)

print t0
print r0

print

print t1
print r1

print

t0h = np.hstack((t0, [1]))

a0 = tft.quaternion_matrix(r0)
a0[:, 3] = t0h

t1h = np.hstack((t1, [1]))
a1 = tft.quaternion_matrix(r1)
a1[:, 3] = t1h

print "pose 0"
print a0

print
print "pose 1"
print a1

a01 = np.dot(tft.inverse_matrix(a0), a1)

print
print "pose1 relative to pose0 (p0->p1)"
print a01

print "-----"

r0i = tft.quaternion_inverse(r0)

print mr.affine_to_pose(a0)
