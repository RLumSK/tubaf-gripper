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

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rosnode

class MoveItWrapper(object):
# see: http://docs.ros.org/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
    def __init__(self):
        # we have to load the arm_prefix from our urdf file into the namespace of an anonymous node started
        # by moveit_commander. Therefore we get all the nodes previous and after its start and select the
        # 'move_group_commander_wrappers_' and pass the parameter into its namespace.
        nodes1 = set(rosnode.get_node_names())
        moveit_commander.roscpp_initialize(sys.argv)
        nodes2 = set(rosnode.get_node_names())

        params = rospy.get_param("/tbf_gripper_autonomy_controller/kinematics")
        for n in nodes2 - nodes1:
            if n.startswith("/move_group_commander_wrappers_"):
                rospy.set_param(n+"/", params)

        group_name = rospy.get_param("~group_name", "UR5")  # name in the kinematics.yaml NOT SRDF
        planned_path_publisher = rospy.get_param("~planned_path_publisher", "/move_group/display_planned_path")
        self.ee_links = rospy.get_param("~ee_links", [])

        self.scene = moveit_commander.PlanningSceneInterface()
        self.commander = moveit_commander.RobotCommander()  # controller of the robot (arm)
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.hand_group = moveit_commander.MoveGroupCommander("Hand")
        self.display_trajectory_publisher = rospy.Publisher(planned_path_publisher, moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=1)
        self.plan = None
        self.group.set_planner_id(rospy.get_param("~planner", "KPIECEkConfigDefault"))
        self.group.set_goal_position_tolerance(rospy.get_param("~goal_position_tolerance", 0.001))
        self.group.set_goal_orientation_tolerance(rospy.get_param("~goal_orientation_tolerance", 0.001))
        self.group.set_goal_joint_tolerance(rospy.get_param("~goal_joint_tolerance", 0.001))
        self.group.set_goal_tolerance(rospy.get_param("~goal_tolerance", 0.001))
        self.group.set_planning_time(rospy.get_param("~planning_time", 30))
        self.group.set_num_planning_attempts(rospy.get_param("~planning_attempts", 100))
        self.group.allow_looking(rospy.get_param("~allow_looking", False))
        self.group.allow_replanning(rospy.get_param("~allow_replanning", False))
        # [minX, minY, minZ, maxX, maxY, maxZ]
        self.group.set_workspace(ws=[-2, -1, -0.40, 0, 1, 1.6])

        # Some information
        rospy.logdebug("--- RobotCommander Info ---")
        rospy.logdebug("MoveItWrapper(): Robot Links: %s", self.commander.get_link_names())
        rospy.logdebug("MoveItWrapper(): Robot Groups: %s", self.commander.get_group_names())
        rospy.logdebug("MoveItWrapper(): Robot Group: %s", self.commander.get_group(group_name))
        rospy.logdebug("MoveItWrapper(): Planning Frame: %s", self.commander.get_planning_frame())
        rospy.logdebug("--- MoveGroupCommander Info ---")
        rospy.logdebug("MoveItWrapper(): group_name: %s", group_name)
        rospy.logdebug("MoveItWrapper(): group_ee: %s", self.group.get_end_effector_link())
        # rospy.logdebug("MoveItWrapper(): Robot State: %s", self.commander.get_current_state())
        rospy.logdebug("MoveItWrapper(): Planing Frame: %s", self.group.get_planning_frame())
        rospy.logdebug("MoveItWrapper(): Path Constraints: %s", self.group.get_path_constraints())
        rospy.logdebug("MoveItWrapper(): Goal tolerance (joints, position, orientation): %s", self.group.get_goal_tolerance())
        rospy.logdebug("MoveItWrapper(): Planning time: %s", self.group.get_planning_time())

    def plan_to_pose(self, pose_stamped):
        rospy.loginfo("MoveItWrapper.plan_to_pose(): begin planning")
        self.group.clear_pose_targets()
        # rospy.loginfo("MoveItWrapper.plan_to_pose() current state - commander: %s", self.commander.get_current_state())
        # rospy.loginfo("MoveItWrapper.plan_to_pose() current state - group: %s", self.group.get_current_joint_values())

        ## http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a55db2d061bbf73d05b9a06df7f31ea39
        self.group.set_start_state(self.commander.get_current_state())
        self.group.set_joint_value_target(pose_stamped)
        self.plan = self.group.plan()
        if not self.plan.joint_trajectory.points:
            # rospy.loginfo("MoveItWrapper.plan_to_pose(): No plan calculated to pose: \n %s", pose_stamped)
            # rospy.loginfo("MoveItWrapper.plan_to_pose() current state - commander: %s",
            #               self.commander.get_current_state())
            # rospy.loginfo("MoveItWrapper.plan_to_pose(): plan (not successful)\n %s", self.plan)
            return False
        # rospy.loginfo("--Joint Trajectory---")
        # rospy.loginfo("%s", self.plan.joint_trajectory)
        # rospy.loginfo("---------------------")
        rospy.sleep(5)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.commander.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.loginfo("MoveItWrapper.plan_to_pose(): end planning successful")
        return self.plan is not None

    def plan_to_joints(self, rbt_state_msg):
        rospy.loginfo("MoveItWrapper.plan_to_joints(): start planning")
        self.group.clear_pose_targets()
        self.group.set_start_state(self.commander.get_current_state())
        joint_states = self.convert_robot_state_for_group(rbt_state_msg)
        dct_joint_states = self.convert_joint_states_to_dict(joint_states)
        self.group.set_joint_value_target(dct_joint_states)
        # rospy.loginfo("MoveItWrapper.plan_to_joints(): plan to:\n %s", dct_joint_states)
        self.plan = self.group.plan()
        rospy.sleep(5)
        # rospy.loginfo("MoveItWrapper.plan_to_joints(): That's the plan \n %s", self.plan.joint_trajectory)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.commander.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.loginfo("MoveItWrapper.plan_to_joints(): end planning successful")
        return self.plan is not None

    def plan_cartesian(self, waypoints):
        # see: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html#cartesian-paths
        self.group.clear_pose_targets()
        waypoints.insert(0, self.group.get_current_pose().pose)
        (aPlan, fraction) = self.group.compute_cartesian_path(waypoints=waypoints, eef_step=0.01, jump_threshold=0.0)
        self.plan = aPlan
        return self.plan is not None

    def get_current_joints(self):
        return self.commander.get_current_state()

    def move_to_pose(self):
        rospy.loginfo("MoveItWrapper.move_to_pose(): Plan:\n%s", self.plan)
        answer = raw_input("Continue? (y/n) ...")
        if answer == 'y':
            return self.group.go(wait=True)
        else:
            return False

    def convert_robot_state_for_group(self, msg):
        joint_states = msg.joint_state
        # rospy.loginfo("MoveItWrapper.convert_robot_state_for_group():joint_states  %s", joint_states)
        # rospy.loginfo("MoveItWrapper.convert_robot_state_for_group():self.group.get_active_joints()  %s", self.group.get_active_joints())

        positions = []
        velocities = []
        efforts = []
        for name in self.group.get_active_joints():
            idx = joint_states.name.index(name)
            positions.append(joint_states.position[idx])
            # velocities.append(joint_states.velocity[idx])
            # efforts.append(joint_states.effort[idx])

        group_start_state_msg = msg
        group_start_state_msg.joint_state.name = self.group.get_active_joints()
        group_start_state_msg.joint_state.position = positions
        group_start_state_msg.joint_state.velocity = velocities
        group_start_state_msg.joint_state.effort = efforts

        # rospy.loginfo("MoveItWrapper.convert_robot_state_for_group():group_start_state_msg  %s", group_start_state_msg)

        return group_start_state_msg

    def convert_joint_states_to_dict(self, joint_msg):
        dct = {}
        for i in range(0, len(joint_msg.joint_state.name)):
            pair = {joint_msg.joint_state.name[i]: joint_msg.joint_state.position[i]}
            dct.update(pair)
        return dct


    def attach_box(self, name="a_box", pose=None, size =(0.15, 0.15, 0.15), touch_links=[]):
        lst = self.ee_links
        lst.extend(touch_links)
        self.scene.attach_box(self.group.get_end_effector_link(), name, pose=pose, size=size, touch_links=lst)

    def remove_attached_object(self, link, name = None):
        self.scene.remove_attached_object(link=link, name=name)

if __name__ == '__main__':
    rospy.init_node("tubaf_grasping_arm", anonymous=False)
    wrapper = MoveItWrapper()
    wrapper.group.set_start_state_to_current_state();
    rospy.sleep(1.0)
    rospy.loginfo("@main current state: %s", wrapper.group.get_current_joint_values())

    pose_st = geometry_msgs.msg.PoseStamped()
    pose_st.header.frame_id = "/gripper_ur5_ee_link"
    pose_st.pose.position.x = 0.331500108757
    pose_st.pose.position.y = 4.63908395643e-07
    pose_st.pose.position.z = 0.078629522774
    pose_st.pose.orientation.x = 0.706717513364
    pose_st.pose.orientation.y = 0.707280545264
    pose_st.pose.orientation.z = 0.0123358400388
    pose_st.pose.orientation.w = -0.0123455921317

    wrapper.plan_to_pose(pose_st)
    rospy.spin()
