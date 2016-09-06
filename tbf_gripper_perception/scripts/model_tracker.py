#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from urdf_parser_py.urdf import URDF

if __name__ == '__main__':
    rospy.init_node("model_tracker")

    model_description = rospy.get_param("~model_description",None)

    if model_description is None:
        rospy.logfatal("empty model_description. Please load urdf model of object to track as ~model_description!")
        exit(-2)

    model = URDF.from_xml_string(model_description)

    print model

