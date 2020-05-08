#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, TU Bergakademie Freiberg
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

"""@package autonomy
This package gives is made to handle provide features supporting autonomy for the Julius robot.
@author: Steve Grehl
"""

import signal
import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import message_filters

import grasping.hand
import numpy as np
import thread

import abc, six


class InterruptError(Exception):
    def __init__(self, *args, **kwargs):
        super(InterruptError, self).__init__(*args, **kwargs)


def pos2str(pos):
    rad = np.deg2rad(pos)
    rad = map(str, rad)
    return "[" + ", ".join(rad) + "]"


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def play_service():
    """
    The new UR driver demands triggering play on the dashboard before work on a new program
    :return: success or not
    :rtype: bool
    """
    from std_srvs.srv import Trigger, TriggerResponse
    # New UR Driver
    # rosservice call /ur_hardware_interface/dashboard/play
    rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
    try:
        play = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        ret_value = play()  # type: TriggerResponse
        rospy.logdebug("MoveTask.play_service(): %s" % ret_value)
        return ret_value.success
    except rospy.ServiceException as e:
        rospy.logerr("MoveTask.move_wait(): dashboard play service call failed: %s" % e)


@six.add_metaclass(abc.ABCMeta)
class MoveTask(object):
    """
    Abstract task class for the UR5
    """
    def __init__(self, js_t=None, bu_pos=None, ltcp_s=None, ltcp_a=None, j_s=None, j_a=None):
        """
        Overloaded constructor assuming the parameter are passed here instead of using the ROS parameter server
        :param js_t: joint state topic
        :type js_t: basestring
        :param bu_pos: backup pose joint values
        :type bu_pos: list(float)
        :param ltcp_s: linear tool center point speed
        :type ltcp_s: float
        :param ltcp_a: linear tool center point acceleration
        :type ltcp_a: float
        :param j_s: joint speed
        :type j_s: float
        :param j_a: joint acceleration
        :type j_a: float
        """
        if js_t is None:
            js_t = rospy.get_param(param_name="~joint_states_topic", default="/ur5/joint_states")
        self.joint_states_topic = js_t
        rospy.logdebug("MoveTask.__init__(): joint_states_topic = %s by given %s" % (self.joint_states_topic, js_t))
        self._joint_sub = message_filters.Subscriber(self.joint_states_topic, JointState)
        self.joint_cache = message_filters.Cache(self._joint_sub, 10)

        self.program_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=1)
        self.move_dur = 1.0
        self.exec_thread = None

        self.waypoints = None
        if bu_pos is None:
            bu_pos = rospy.get_param("~backup_pos")
        self.backup_pose = bu_pos
        if ltcp_s is None:
            ltcp_s = rospy.get_param("~linear_tcp_speed", 60.0)
        self.l_arm_speed = ltcp_s
        if ltcp_a is None:
            ltcp_a = rospy.get_param("~linear_tcp_acceleration", 20.0)
        self.l_arm_acceleration = ltcp_a
        if j_s is None:
            j_s = rospy.get_param("~joint_speed", 10.0)
        self.j_arm_speed = j_s
        if j_a is None:
            j_a = rospy.get_param("~joint_acceleration", 5.0)
        self.j_arm_acceleration = j_a

    def initialise(self):
        """
        Initialise by opening the hand and moving to a given backup_pose
        """
        rospy.loginfo("MoveTask.initialise(): Move to backup_pose...")
        self.move_wait(self.backup_pose, v=45, a=20)
        rospy.loginfo("MoveTask.initialise(): Init done")

    def run_as_process(self, function):
        """
        Run a given function as Thread process
        :param function: function to execute
        :type function: function
        :return: -
        :rtype: -
        """
        if self.exec_thread is not None:
            rospy.logwarn("MoveTask.run_as_process() self.exec_thread=%s", self.exec_thread)
            rospy.sleep(.5)
            self.run_as_process(function)
            return
        rospy.loginfo("MoveTask.run_as_process(): Starting %s" % function)
        self.exec_thread = thread.start_new_thread(function, (self,))
        rospy.loginfo("MoveTask.run_as_process(): Finished %s" % function)
        self.exec_thread = None

    def _convert_joint_States(self, js):
        """
        Converter function - Former Callback for the joint_states topic of the UR5
        :param js: joint states
        :type js: double
        :return: most recent joint states in radian
        :rtype: list(float)
        """
        pp = list(js.position)
        # Resolve mismatch of joint positions between older firmware version and newer
        pp[0] = js.position[js.name.index('gripper_ur5_shoulder_pan_joint')]
        pp[1] = js.position[js.name.index('gripper_ur5_shoulder_lift_joint')]
        pp[2] = js.position[js.name.index('gripper_ur5_elbow_joint')]
        pp[3] = js.position[js.name.index('gripper_ur5_wrist_1_joint')]
        pp[4] = js.position[js.name.index('gripper_ur5_wrist_2_joint')]
        pp[5] = js.position[js.name.index('gripper_ur5_wrist_3_joint')]
        vls = np.asarray(pp)
        vls[np.abs(vls) < 0.01] = 0
        pp = vls.tolist()
        rospy.logdebug("MoveTask._convert_joint_States(): pp=%s", pp)

        return pp

    def move_wait(self, pose, goal_tolerance=1.0, v=None, a=None, t=0, r=0, move_cmd="movej"):
        """
        Move a UR5 to a given pose using the URScript interface
        :param pose: target pose as joint values
        :type pose: list(6x double)
        :param goal_tolerance: tolerance for the target (degree)
        :type goal_tolerance: double
        :param v: velocity for the joint or tcp in deg/s or mm/s
        :type v: double
        :param a: acceleration for the joint values or tcp in deg/s2 or mm/s2
        :type a: double
        :param t: time
        :type t: double
        :param r: blend radius
        :type r: double
        :param move_cmd: move command from URScript
        :type move_cmd: str [movej, movel, movep]
        :return: -
        :rtype: -
        """
        rospy.loginfo("MoveTask.move_wait(): %s to:\n %s", move_cmd,  str(pose))
        program = move_cmd + "(%s" % pos2str(pose)
        if a is not None:
            if move_cmd == "movej":
                program += ", a=%f" % np.deg2rad(a)
            elif move_cmd == "movel":
                program += ", a=%f" % a
        else:
            if move_cmd == "movej":
                program += ", a=%f" % np.deg2rad(self.j_arm_acceleration)
            elif move_cmd == "movel":
                program += ", a=%f" % self.l_arm_acceleration
        if v is not None:
            if move_cmd == "movej":
                program += ", v=%f" % np.deg2rad(v)
            elif move_cmd == "movel":
                program += ", v=%f" % v
        else:
            if move_cmd == "movej":
                program += ", v=%f" % np.deg2rad(self.j_arm_speed)
            elif move_cmd == "movel":
                program += ", v=%f" % self.l_arm_speed
        if t is not None:
            program += ", t=%f" % t
        if t is not None:
            program += ", r=%f" % r
        program += ")"
        rospy.loginfo("MoveTask.move_wait(): %s ", program)

        self.program_pub.publish(program)
        while not play_service():
            rospy.loginfo("MoveTask.move_wait(): wait for 'ros_control' @ Teach Pendant to be ready")
            rospy.sleep(1.0)
        while True:
            tmp = self.joint_cache.getLast()
            if tmp is None:
                rospy.logwarn("MoveTask.move_wait(): No joint states received from driver at: %s",
                              self.joint_states_topic)
                rospy.sleep(5.0)
                continue
            cur_pos = np.rad2deg(np.asarray(self._convert_joint_States(tmp))).tolist()
            max_dst = np.max(np.abs(np.subtract(pose, cur_pos)))
            rospy.logdebug("MoveTask.move_wait(): cur_pos =%s", cur_pos)
            rospy.logdebug("MoveTask.move_wait(): des_pos =%s", pose)
            rospy.logdebug("MoveTask.move_wait(): max_dst = %s (tol=%s)", max_dst, goal_tolerance)
            if max_dst < goal_tolerance:
                break
            rospy.sleep(1.0)
        rospy.loginfo("MoveTask.move_wait(): Reached pose")


@six.add_metaclass(abc.ABCMeta)
class GraspTask(MoveTask):
    """
    Abstract class for set tasks
    """
    def __init__(self, js_t=None, bu_pos=None, ltcp_s=None, ltcp_a=None, j_s=None, j_a=None):
        """
        Overloaded constructor assuming the parameter are passed here instead of using the ROS parameter server
        :param js_t: joint state topic
        :type js_t: basestring
        :param bu_pos: backup pose joint values
        :type bu_pos: list(float)
        :param ltcp_s: linear tool center point speed
        :type ltcp_s: float
        :param ltcp_a: linear tool center point acceleration
        :type ltcp_a: float
        :param j_s: joint speed
        :type j_s: float
        :param j_a: joint acceleration
        :type j_a: float
        """
        super(GraspTask, self).__init__(js_t, bu_pos, ltcp_s, ltcp_a, j_s, j_a)
        # ROS Anbindung
        self.hand_controller = grasping.hand.AdvancedHandController()
        rospy.loginfo("Task.py: GraspTask(): initilized hand")

        # rospy.sleep(0.5)
        # self.initialise()
        # super(GraspTask, self).run_as_process(self.initialise)

    @abc.abstractmethod
    def perform(self):
        """
        perfom a given set task
        :return: -
        :rtype: -
        """
        pass

    def initialise(self):
        """
        Initilise by opening the hand and moving to a given backup_pose
        :return: -
        :rtype: -
        """
        super(GraspTask, self).initialise()
        # Hand
        # rospy.sleep(3.0)
        rospy.logdebug("GraspTask.initialise(): Initialized arm now setup the hand")
        self.hand_controller.openHand()
        # rospy.loginfo("GraspTask.initialisation(): Wait 20s for the hand to initialise")
        # rospy.sleep(20.0)
        rospy.loginfo("GraspTask.initialise(): Init done")
        # self.exec_thread = None

    @abc.abstractmethod
    def start(self):
        """
        Start the water Sample Task
        :return: -
        :rtype: -
        """
        rospy.loginfo("GraspTask.start():")
        super(GraspTask, self).run_as_process(GraspTask.perform)


@six.add_metaclass(abc.ABCMeta)
class SetTask(MoveTask):
    """
    Abstract class for set tasks
    """
    def __init__(self):
        """
        Default constructor, start ROS, hand_model and demo_monitoring
        """
        MoveTask.__init__(self)
        rospy.logwarn("SetTask.__init__(): This class has been renamed to 'GraspTask' please rename its usage too")


if __name__ == '__main__':
    rospy.init_node("Task")
    obj = GraspTask()
    while obj.exec_thread is not None:
        rospy.sleep(0.5)
    obj.start()
    rospy.spin()
