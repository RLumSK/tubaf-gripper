#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose, PoseStamped
from collections import deque

# all configuration goes here
class cfg:
    num_samples = 10

# singleton class for global variables
class g:
    marker_sub = None
    transforms = dict()

def on_markers(markers):
    markers = AlvarMarkers()


    pass

def on_optimize():
    pass

if __name__ == '__main__':





    g.marker_sub = rospy.Subscriber("/ar_pose_marker",AlvarMarkers,on_markers)
    g.optim_timer = rospy.Timer(rospy.Time(2),on_optimize)
    rospy.spin()
    pass