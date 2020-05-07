#!/usr/bin/python
# Software License Agreement (MIT License)
#
# Copyright (c) 2019, TU Bergakademie Freiberg
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

from autonomy.Task import MoveTask


class ScanTask(MoveTask):
    """
    Handle equipment using the gripper unit of Julius
    """
    def __init__(self):
        """
        Overloaded constructor assuming the parameter are passed using the ROS parameter server
        """
        # @All parameters were imported@
        super(ScanTask, self).__init__(js_t=rospy.get_param("~arm/joint_states_topic"),
                                       bu_pos=rospy.get_param("~arm/backup_joint_values"),
                                       ltcp_s=rospy.get_param("~arm/linear_tcp_speed"),
                                       ltcp_a=rospy.get_param("~arm/linear_tcp_acceleration"),
                                       j_s=rospy.get_param("~arm/joint_speed"),
                                       j_a=rospy.get_param("~arm/joint_acceleration"))
        # # Init Moveit
        # self.tf_listener = tf.TransformListener(rospy.Duration.from_sec(15.0))
        # self.moveit = MoveitInterface("~moveit", self.tf_listener)  # type: MoveitInterface
        # rospy.loginfo("ScanTask.__init__(): Moveit initialized")

        # Static joint values for specific well known poses
        self.backup_joint_values = rospy.get_param("~arm/backup_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.home_joint_values = rospy.get_param("~arm/home_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.watch_joint_values = rospy.get_param("~arm/watch_joint_values", [-180, -90, 0.0, -90, 0.00, 0.0])
        self.target_values = rospy.get_param("~lst_scan_joint_values", [self.home_joint_values, self.watch_joint_values]
                                             )
        print self.target_values
        self.rest_duration = rospy.Duration.from_sec(rospy.get_param("~rest_duration", 2.0))
        rospy.loginfo("ScanTask.__init__(): initialized")

    # def move_wait(self, target, move_cmd="movej"):
    #     """
    #     Overrides super method
    #     :param target:
    #     :type target:
    #     :param move_cmd:
    #     :type move_cmd:
    #     :return:
    #     :rtype:
    #     """
    #     if move_cmd == "movej":
    #         super(ScanTask, self).move_wait(target, goal_tolerance=1.0, v=self.j_arm_speed, a=self.j_arm_acceleration,
    #                                         move_cmd=move_cmd)
    #     elif move_cmd == "movel":
    #         super(ScanTask, self).move_wait(target, goal_tolerance=1.0, v=self.l_arm_speed, a=self.l_arm_acceleration,
    #                                          move_cmd=move_cmd)

    def perform(self):
        """
        Scan Task:
            0. Close Hand
            i.      Move to Scan Pose in list
            i+1.    Rest fot a set duration
            n. Move to starting position
        :return: -
        :rtype: -
        """
        for i in range(len(self.target_values)):
            target = self.target_values[i]
            rospy.loginfo("ScanTask.perform(): target %s", target)
            self.move_wait(target, move_cmd="movel")
            # self.moveit.move_to_target(target, str(i+1))
            rospy.sleep(self.rest_duration)
        self.move_wait(self.target_values[0], move_cmd="movel")
        rospy.loginfo("ScanTask.perform(): Finished")

    def start(self):
        """
        Start the equipment handle task
        :return: -
        :rtype: -
        """
        self.move_wait(self.home_joint_values)
        self.move_wait(self.target_values[0])
        while not rospy.is_shutdown():
            self.perform()
        self.move_wait(self.home_joint_values)


if __name__ == '__main__':
    rospy.init_node("ScanTask", log_level=rospy.INFO)
    obj = ScanTask()
    rospy.sleep(1.0)
    obj.start()
