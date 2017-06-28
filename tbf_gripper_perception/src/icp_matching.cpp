#include <ros/ros.h>


#include "tbf_gripper_perception/icpobjectsearch.h"

using namespace std;


int main(int argc, char** argv)
{
    string model_name, pcl_topic;
    //ROS
    ros::init(argc, argv, "surface_matching");

    ros::NodeHandle nh("~");
    nh.param<std::string>("ply_model_name", model_name, "wlan_blender");
    nh.param<std::string>("pcl_topic", pcl_topic, "/ork/obj_clouds");
    ROS_INFO_STREAM("[icp_matching.main] ply_model_name " << model_name);
    ROS_INFO_STREAM("[icp_matching.main] pcl_topic " << pcl_topic);
    IcpObjectSearch icp_search(nh, model_name, pcl_topic);

    ros::spin();
    return 0;

}
