#include "ros/ros.h"
#include <ros/console.h>
#include "geometry_msgs/PoseStamped.h"

#include "ur_kinematics/ikfast.h"
#include "ur_kinematics/ur_kin.h"
#include "ur_kinematics/ur_moveit_plugin.h"
#include <moveit_msgs/MoveItErrorCodes.h>


double get_degrees(double input)
{
    return input * 180.0 / M_PI;
}

void print_solution(std::vector<double> solution){
    std::vector<double>::iterator it;
    for(it = solution.begin(); it != solution.end(); ++it){
        ROS_INFO("%f, ",get_degrees(*it));
    }
    ROS_INFO("-");
}

int main(int argc, char **argv){
    /*start node*/
    ros::init(argc, argv, "test_ik_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    pn.setParam("UR5/arm_prefix", "gripper_ur5_");

    /*setup pose stamped*/
    geometry_msgs::PoseStamped test_pose;
    test_pose.header.frame_id = "/gripper_ur5_ee_link";
    test_pose.pose.position.x = 0.331500108757;
    test_pose.pose.position.y = 4.63908395643e-07;
    test_pose.pose.position.z = 0.078629522774;
    test_pose.pose.orientation.x = 0.706717513364;
    test_pose.pose.orientation.y = 0.707280545264;
    test_pose.pose.orientation.z = 0.0123358400388;
    test_pose.pose.orientation.w = -0.0123455921317;
    std::vector<double> seed;
    /*
    seed.push_back(0.07993608);
    seed.push_back(-1.2864822);
    seed.push_back(1.4622368);
    seed.push_back(-1.7135643);
    seed.push_back(-1.5353661);
    seed.push_back(0.71959925);
    */
    seed.push_back(0.0);
    seed.push_back(0.0);
    seed.push_back(0.0);
    seed.push_back(0.0);
    seed.push_back(0.0);
    seed.push_back(0.0);

    /*ik*/
    using namespace ur_kinematics;
    URKinematicsPlugin ik_plugin;
    if(!ik_plugin.initialize("/robot_description", "UR5", "gripper_ur5_base_link", "gripper_ur5_ee_link", 0.1)){
        ROS_WARN("@test_ik.cpp: Couldn't initilize ur_movit_ik_plugin");
        return -1;
    }

    /*joint values for pose*/
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error;
    if(!ik_plugin.getPositionIK(test_pose.pose, seed, solution, error)){
        ROS_WARN("@test_ik.cpp: getPositionIK failed");
        return -1;
    }
    print_solution(solution);

    if(!ik_plugin.searchPositionIK(test_pose.pose, seed, 10.0, solution, error)){
        ROS_WARN("@test_ik.cpp: getPositionIK failed");
        return -1;
    }
    print_solution(solution);



    ros::spin();
    // moveit::planning_interface::MoveGroup group("right_arm");
}
