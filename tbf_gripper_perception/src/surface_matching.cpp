#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include "drostobjectsearch.h"

using namespace std;


int main(int argc, char** argv)
{
    string model_path, model_name, modelFileName, pcl_topic;
    //ROS
    ros::init(argc, argv, "surface_matching");

    // welcome message
    ROS_INFO_STREAM(endl << "****************************************************" << endl <<
                    "* Surface Matching demonstration : demonstrates the use of surface matching using point pair features." << endl <<
                    "* The sample loads a model and a scene, where the model lies in a different pose than the training.\n* It then trains the model and searches for it in the"
                    " input scene. The detected poses are further refined by ICP\n* and printed to the "
                    " standard output." << endl <<
                    "****************************************************" << endl);

    ros::param::param<std::string>("ply_model_path", model_path, "/home/grehl/robots/julius_util_ws/src/tubaf_gripper/tbf_gripper_perception/meshes/");
    ros::param::param<std::string>("ply_model_name", model_name, "wlan_box_pv");
    ros::param::param<std::string>("pcl_topic", pcl_topic, "/camera/depth/points");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Using file: " << model_path + model_name + ".ply");
    DrostObjectSearch perception(nh, model_path + model_name + ".ply", pcl_topic);

    ros::spin();
    return 0;

}
