#!/usr/bin/python
# coding=utf-8
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, TU Bergakademie Freiberg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# Author: grehl

import rospy
from autonomy.EquipmentTask import EquipmentTask
from evaluation.EvaluateEquipmentTask import EquipmentTask as Evaluation
from tubaf_tools.help import play_sound


if __name__ == '__main__':
    rospy.init_node("EquipmentTask", log_level=rospy.DEBUG)
    et = EquipmentTask(evaluation=Evaluation())  # type: EquipmentTask
    et.hand_controller.openHand()
    rospy.sleep(rospy.Duration(1))
    et.hand_controller.closeHand(continue_image_service=False)

    str_exp = rospy.get_param("~experiment", "set_and_pick")
    object_names = rospy.get_param("~object", "Henkel,Knauf,Kugel").split(",")
    if type(object_names) is str:
        object_names = [object_names]
    objects = []
    for obj in et.lst_equipment:
        if obj.name in object_names:
            objects.append(obj)
            rospy.loginfo("[exp_set_ssb] Adding %s" % obj.name)
    nn = rospy.get_name()

    for obj in objects:
        if not et.select_equipment(obj.name):
            rospy.logwarn("[exp_set_ssb as %s] Didn't select %s - not loaded in EquipmentTask context" % (nn, obj.name))
            continue
        try:
            if "set" in str_exp:
                rospy.loginfo("[exp_set_ssb as %s] Set %s" % (nn, obj.name))
                et.start()
            if "pick" in str_exp:
                rospy.loginfo("[exp_set_ssb as %s] Pick %s" % (nn, obj.name))
                et.pick_after_place(obj)
            play_sound("finish")
        except Exception as ex:
            rospy.logerr(ex.message)
            play_sound("error")
        finally:
            if et.evaluation:
                secs = rospy.Time.now().secs
                et.evaluation.save_as_bag("~/bags" + nn + "_" + str_exp + "_" + obj.name + str(secs)[6:] + ".bag")
                et.evaluation = Evaluation()
                et.moveit.evaluation = et.evaluation
        rospy.loginfo("[exp_set_ssb] Finished %s" % obj.name)
        play_sound("world")
    rospy.spin()
